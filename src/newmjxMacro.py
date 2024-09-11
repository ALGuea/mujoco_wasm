import re

def parse_references(filename):
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
    array_types = {
        "int": "Int32Array",
        "mjtNum": "Float64Array",
        "float": "Float32Array",
        "mjtByte": "Uint8Array",
        "char": "Uint8Array",
        "uintptr_t": "BigUint64Array"
    }
    return array_types.get(field_type.rstrip('*'), "Float64Array")

def parse_mjxmacro(filename):
    with open(filename, 'r') as f:
        content = f.read()

    model_definitions = []
    model_bindings = []
    data_definitions = []
    data_bindings = []
    enums = []

    # Helper function to determine if the field is a pointer or listed in the pointers macro
    def is_pointer_or_in_pointers_macro(field_name, pointers_macro_content):
        return any(field_name in line for line in pointers_macro_content)

    # Parse MJMODEL_INTS and MJMODEL_POINTERS
    model_macros = re.findall(r'#define MJMODEL_(\w+)\s+\\([\s\S]*?)\n\n', content)
    model_pointers_macro = next((macro_content for macro_name, macro_content in model_macros if macro_name == 'POINTERS'), '').strip().split('\n')

    print(f"Found {len(model_macros)} MJMODEL macros")
    for macro_name, macro_content in model_macros:
        print(f"Processing MJMODEL_{macro_name}:")
        print(macro_content)
        lines = macro_content.strip().split('\n')
        for line in lines:
            match = re.match(r'\s*X\s*\(\s*(\w+)\s*,\s*(\w+)\s*([,\)])\s*(\w+)?', line)
            if match:
                type_, name, _, size = match.groups()
                size = size if size else '1'
                if type_.endswith('*') or is_pointer_or_in_pointers_macro(name, model_pointers_macro):
                    model_definitions.append(f"val  {name}() const {{ return val(typed_memory_view(m->{size}, m->{name})); }}")
                    model_bindings.append(f'.property("{name}", &Model::{name})')
                else:
                    model_definitions.append(f"{type_}  {name}() const {{ return m->{name}; }}")
                    model_bindings.append(f'.property("{name}", &Model::{name})')

    print(f"Generated {len(model_definitions)} model definitions")

    # Parse MJDATA_POINTERS
    data_macro = re.search(r'#define MJDATA_POINTERS\s+\\([\s\S]*?)\n\n', content)
    data_pointers_macro_content = data_macro.group(1).strip().split('\n') if data_macro else []

    if data_macro:
        lines = data_macro.group(1).strip().split('\n')
        for line in lines:
            match = re.match(r'\s*X\s*\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\)', line)
            if match:
                type_, name, size1, size2 = match.groups()
                if type_.endswith('*') or is_pointer_or_in_pointers_macro(name, data_pointers_macro_content):
                    data_definitions.append(f"val  {name}() const {{ return val(typed_memory_view(_model->ptr()->{size1} * {size2}, _state->ptr()->{name})); }}")
                    data_bindings.append(f'.property("{name}", &Simulation::{name})')
                else:
                    data_definitions.append(f"{type_}  {name}() const {{ return _state->ptr()->{name}; }}")
                    data_bindings.append(f'.property("{name}", &Simulation::{name})')

    print(f"Generated {len(data_definitions)} data definitions")

    # Parse enums
    enum_matches = re.findall(r'typedef enum\s*_?mj(\w+)\s*{([\s\S]*?)}\s*mj\w+;', content)
    print(f"Found {len(enum_matches)} enums")
    for enum_name, enum_content in enum_matches:
        enum_values = re.findall(r'(\w+)\s*[,=]', enum_content)
        enum_def = f"enum_<mj{enum_name}>(\"mj{enum_name}\")\n"
        for value in enum_values:
            enum_def += f'    .value("{value}", mj{enum_name}::{value})\n'
        enum_def += ";"
        enums.append(enum_def)

    return model_definitions, model_bindings, data_definitions, data_bindings, enums

def generate_main_cc(template_file, output_file, model_defs, model_bindings, data_defs, data_bindings, enums):
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
if __name__ == "__main__":
    mjxmacro_file = "include/mujoco/mjxmacro.h"
    template_file = "src/main.template.cc"
    output_file = "src/main.genned.cc"

    
    # Usage
    
    model_definitions, model_bindings, data_definitions, data_bindings, enums = parse_mjxmacro(mjxmacro_file)
    generate_main_cc(template_file, output_file, model_definitions, model_bindings, data_definitions, data_bindings, enums)


    print(f"Generated {output_file}")
