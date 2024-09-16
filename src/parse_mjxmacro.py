import re

def parse_references(filename):
    """
    Parses the struct types in the file to get type and array size information.
    """
    with open(filename, 'r') as f:
        content = f.read()
    
    types = {}
    structs = re.findall(r'struct\s+(\w+)_\s*{([\s\S]*?)};', content)
    for struct_name, struct_content in structs:
        fields = re.findall(r'\s*(\w+(?:\s*\*)?)\s+(\w+)(?:\[([^\]]+)\])?;', struct_content)
        for field_type, field_name, array_size in fields:
            types[f"{struct_name}.{field_name}"] = (field_type, array_size)
    
    return types

def get_array_type(field_type):
    """
    Maps C++ types to TypeScript array types.
    """
    array_types = {
        "int": "Int32Array",
        "mjtNum": "Float64Array",
        "float": "Float32Array",
        "mjtByte": "Uint8Array",
        "char": "Uint8Array",
        "uintptr_t": "BigUint64Array"
    }
    return array_types.get(field_type.rstrip('*'), "Float64Array")

def parse_mjxmacro(filename, expose_list):
    """
    Parses the mjxmacro.h file for model and data macro definitions, generates C++ and TypeScript bindings.
    """
    with open(filename, 'r') as f:
        content = f.read()

    model_definitions = []
    model_bindings = []
    data_definitions = []
    data_bindings = []
    enums = []

    # Helper function to determine if the field is in the expose list
    def is_in_expose_list(field_name):
        return field_name in expose_list

    # Helper function to determine if the field is a pointer or listed in the pointers macro
    def is_pointer_or_in_pointers_macro(field_name, pointers_macro_content):
        return any(field_name in line for line in pointers_macro_content)

    # Parse MJMODEL_INTS and MJMODEL_POINTERS
    model_macros = re.findall(r'#define MJMODEL_(\w+)\s+\\([\s\S]*?)\n\n', content)
    model_pointers_macro = next((macro_content for macro_name, macro_content in model_macros if macro_name == 'POINTERS'), '').strip().split('\n')

    for macro_name, macro_content in model_macros:
        lines = macro_content.strip().split('\n')
        for line in lines:
            match = re.match(r'\s*X\s*\(\s*(\w+)\s*,\s*(\w+)\s*([,\)])\s*(\w+)?', line)
            if match:
                type_, name, _, size = match.groups()
                size = size if size else '1'
                if not is_in_expose_list(name):
                    continue  # Skip elements not in the expose list
                
                # C++ Binding Generation
                if type_.endswith('*') or is_pointer_or_in_pointers_macro(name, model_pointers_macro):
                    model_definitions.append(f"val  {name}() const {{ return val(typed_memory_view(m->{size}, m->{name})); }}")
                    model_bindings.append(f'.property("{name}", &Model::{name})')
                else:
                    model_definitions.append(f"{type_}  {name}() const {{ return m->{name}; }}")
                    model_bindings.append(f'.property("{name}", &Model::{name})')

    # Parse MJDATA_POINTERS
    data_macro = re.search(r'#define MJDATA_POINTERS\s+\\([\s\S]*?)\n\n', content)
    data_pointers_macro_content = data_macro.group(1).strip().split('\n') if data_macro else []

    if data_macro:
        lines = data_macro.group(1).strip().split('\n')
        for line in lines:
            match = re.match(r'\s*X\s*\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\)', line)
            if match:
                type_, name, size1, size2 = match.groups()
                if not is_in_expose_list(name):
                    continue  # Skip elements not in the expose list

                # C++ Binding Generation
                if type_.endswith('*') or is_pointer_or_in_pointers_macro(name, data_pointers_macro_content):
                    data_definitions.append(f"val  {name}() const {{ return val(typed_memory_view(_model->ptr()->{size1} * {size2}, _state->ptr()->{name})); }}")
                    data_bindings.append(f'.property("{name}", &Simulation::{name})')
                else:
                    data_definitions.append(f"{type_}  {name}() const {{ return _state->ptr()->{name}; }}")
                    data_bindings.append(f'.property("{name}", &Simulation::{name})')

    # Parse enums
    enum_matches = re.findall(r'typedef enum\s*_?mj(\w+)\s*{([\s\S]*?)}\s*mj\w+;', content)
    for enum_name, enum_content in enum_matches:
        enum_values = re.findall(r'(\w+)\s*[,=]', enum_content)
        enum_def = f"enum_<mj{enum_name}>(\"mj{enum_name}\")\n"
        for value in enum_values:
            enum_def += f'    .value("{value}", mj{enum_name}::{value})\n'
        enum_def += ";"
        enums.append(enum_def)

    return model_definitions, model_bindings, data_definitions, data_bindings, enums

def generate_main_cc(template_file, output_file, model_defs, model_bindings, data_defs, data_bindings, enums):
    """
    Generate the main.genned.cc file by injecting dynamically generated C++ bindings.
    """
    with open(template_file, 'r') as f:
        template = f.readlines()

    output_lines = []
    for line in template:
        output_lines.append(line)
        if "// MJMODEL_DEFINITIONS" in line:
            output_lines.extend(model_def + '\n' for model_def in model_defs)
        elif "// MJMODEL_BINDINGS" in line:
            output_lines.extend(model_binding + '\n' for model_binding in model_bindings)
        elif "// MJDATA_DEFINITIONS" in line:
            output_lines.extend(data_def + '\n' for data_def in data_defs)
        elif "// MJDATA_BINDINGS" in line:
            output_lines.extend(data_binding + '\n' for data_binding in data_bindings)
        elif "// MODEL_ENUMS" in line:
            output_lines.extend(enum + '\n' for enum in enums)

    with open(output_file, 'w') as f:
        f.writelines(output_lines)

    print(f"Generated {output_file} with {len(output_lines)} lines")

# **TypeScript Declaration File Generation**
def generate_d_ts(template_file, output_file, model_defs, data_defs):
    """
    Generate the TypeScript bindings .d.ts file by injecting dynamically generated TypeScript bindings.
    """
    with open(template_file, 'r') as f:
        template = f.readlines()

    output_lines = []
    for line in template:
        output_lines.append(line)
        # Inject model property declarations in the correct place
        if "// MODEL_INTERFACE" in line:
            output_lines.extend(f"    {get_ts_name(model_def)}: {get_ts_type(model_def)};\n" for model_def in model_defs)
        # Inject data property declarations in the correct place
        if "// DATA_INTERFACE" in line:
            output_lines.extend(f"    {get_ts_name(data_def)}: {get_ts_type(data_def)};\n" for data_def in data_defs)

    with open(output_file, 'w') as f:
        f.writelines(output_lines)

    print(f"Generated {output_file} with {len(output_lines)} lines")

# Helper function to extract TypeScript field name from C++ binding
def get_ts_name(binding):
    """
    Extract the TypeScript-friendly field name from the C++ binding.
    """
    match = re.search(r'\b(\w+)\(', binding)
    return match.group(1) if match else binding

# Helper function to generate TypeScript types
def get_ts_type(binding):
    """
    Determine the TypeScript type based on the C++ binding.
    """
    ts_types = {
        "int": "number",
        "mjtNum": "number",
        "float": "number",
        "mjtByte": "Uint8Array",
        "char": "string",
        "uintptr_t": "bigint"
    }
    return ts_types.get(binding, "Float64Array")

if __name__ == "__main__":
    # Specify the elements you want to expose
    expose_list = ['qpos0', 'qpos', 'qvel', 'xpos', 'xquat', 'ctrl', 'act', 'jnt_qposadr', 'jnt_dofadr', 'qfrc_applied', 'mocap_pos', 'mocap_quat', 'act_dot', 'userdata', 'plugin', 'plugin_data', 'qH']

    mjxmacro_file = "include/mujoco/mjxmacro.h"
    template_file_cc = "src/main.template.cc"
    output_file_cc = "src/main.genned.cc"
    template_file_ts = "src/mujoco_wasm.template.d.ts"
    output_file_ts = "src/mujoco_wasm.genned.d.ts"

    # Run the parsing and file generation
    try:
        model_definitions, model_bindings, data_definitions, data_bindings, enums = parse_mjxmacro(mjxmacro_file, expose_list)
        generate_main_cc(template_file_cc, output_file_cc, model_definitions, model_bindings, data_definitions, data_bindings, enums)
        generate_d_ts(template_file_ts, output_file_ts, model_definitions, data_definitions)
    except FileNotFoundError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    print(f"Generated {output_file_cc} and {output_file_ts}")
