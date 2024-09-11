import sys
import functions
import re  # This line imports the regular expressions module
from ast_nodes import ValueType

auto_gen_lines = {
    "model_definitions": [],
    "model_bindings"   : [],
    "model_typescript" : [],
    "model_enums"      : [],
    "data_definitions": [],
    "data_bindings"    : [],
    "data_typescript"  : [],
    "enums_typescript" : [],
}

parse_mode = (None, None)
types_to_array_types = {"int":"Int32Array", "mjtNum":"Float64Array", "float": "Float32Array", "mjtByte": "Uint8Array", "char": "Uint8Array", "uintptr_t":"BigUint64Array", "size_t":"BigUint64Array"}

def parse_pointer_line(line:str, header_lines:list[str], mj_definitions:list[str], emscripten_bindings:list[str], typescript_definitions:list[str]):
    if "    X   (" in line:
        elements = line.strip("    X   (").split(""")""")[0].strip().split(",")
    elif "    XMJV(" in line:
        elements = line.strip("    XMJV(").split(""")""")[0].strip().split(",")
    elements = [e.strip() for e in elements]
    elements[0] = elements[0].replace("X(", "").replace("XMJV(", "").strip()

    if len(elements) < 2:
        print(f"Warning: Unexpected format in line: {line.strip()}")
        return

    type_, name = elements[:2]
    size = elements[2] if len(elements) > 2 else "1"

    if parse_mode[1] == "model":
        ptr = "m"
        size_ptr = "m"
    else:
        ptr = "_state->ptr()"
        size_ptr = "_model->ptr()"

    if size.startswith("MJ_M("):
        size = f"{size_ptr}->{size[5:-1]}"
    elif size.startswith("MJ_D("):
        size = f"{ptr}->{size[5:-1]}"

    # Reintegrate dynamic val entry creation
    mj_definitions.append(f'  val  {name.ljust(22)} () const {{ return val(typed_memory_view({size_ptr}->{size} * 1, {ptr}->{name} )); }}')

    emscripten_bindings.append(f'      .property("{name}", &{"Model" if parse_mode[1] == "model" else "Simulation"}::{name})')
    for model_line in header_lines:
        if f"{type_}* {name};" in model_line or f"{type_} {name};" in model_line:
            comment = model_line.split("//")[1].strip() if "//" in model_line else ""
            typescript_definitions.append(f"  /** {comment} */")
            break

    typescript_definitions.append(f'  {name.ljust(22)}: {types_to_array_types.get(type_, "any").rjust(12)};')



def parse_int_line(line:str, header_lines:list[str], mj_definitions:list[str], emscripten_bindings:list[str], typescript_definitions:list[str]):
    if "    X   (" in line:
        name = line.strip("    X   (").split(""")""")[0].strip()
    elif "    XMJV(" in line:
        name = line.strip("    XMJV(").split(""")""")[0].strip()
    mj_definitions     .append('  int  '+name.ljust(14)+'() const { return m->'+name.ljust(14)+'; }')
    emscripten_bindings.append('      .property('+('"'+name+'"').ljust(24)+', &Model::'+name.ljust(22)+')')

    # Iterate through the file looking for comments
    for model_line in header_lines:
        if f"int {name};" in model_line:
            comment = model_line.split("//")[1].strip() if "//" in model_line else ""
            typescript_definitions.append(f"  /** {comment} */")
            break

    typescript_definitions.append(f'  {name.ljust(22)}: {"number".rjust(12)};')

def read_file(filename):
    try:
        with open(filename, 'r') as f:
            return f.readlines()
    except IOError:
        print(f"Error: Unable to read file {filename}")
        sys.exit(1)

model_lines = read_file("include/mujoco/mjmodel.h")
data_lines = read_file("include/mujoco/mjdata.h")
mjxmacro_lines = read_file("include/mujoco/mjxmacro.h")

print("Parsing mjxmacro.h file...")  # Debug print

# Parse mjx Macro to get the emscripten bindings and typescript definitions
for line in mjxmacro_lines:
    if "#define MJMODEL_INTS" in line:
        print("Found MJMODEL_INTS")  # Debug print
        parse_mode = ("ints", "model")
    elif "#define MJMODEL_POINTERS" in line:
        print("Found MJMODEL_POINTERS")  # Debug print
        parse_mode = ("pointers", "model")
    elif "#define MJDATA_POINTERS" in line:
        print("Found MJDATA_POINTERS")  # Debug print
        parse_mode = ("pointers", "data")
    
    if parse_mode[0] is not None:
        if line.strip().startswith(("X(", "XMJV(")):
            if parse_mode[0] == "pointers":
                if line.strip().startswith("X   (") or line.strip().startswith("XMJV("):
                    parse_pointer_line(line, 
                                       model_lines if parse_mode[1] == "model" else data_lines, 
                                       auto_gen_lines[parse_mode[1]+"_definitions"], 
                                       auto_gen_lines[parse_mode[1]+"_bindings"], 
                                       auto_gen_lines[parse_mode[1]+"_typescript"])
                else:
                    parse_mode = (None, None)

            if parse_mode[0] == "ints":
                if line.strip().startswith("X   (") or line.strip().startswith("XMJV("):
                    parse_int_line(line, 
                                   model_lines if parse_mode[1] == "model" else data_lines, 
                                   auto_gen_lines[parse_mode[1]+"_definitions"], 
                                   auto_gen_lines[parse_mode[1]+"_bindings"], 
                                   auto_gen_lines[parse_mode[1]+"_typescript"])
                else:
                    parse_mode = (None, None)

print("Finished parsing mjxmacro.h file")  # Debug print

for function in functions.FUNCTIONS:
    param_types = [param.decltype for param in functions.FUNCTIONS[function].parameters]
    name = function[3:] if function != "mj_crb" else function[3:] + "Calculate"
    function_def = "  void "+name.ljust(22)+"() { "+function.ljust(28)+"("
    def_args   = []
    def_params = []
    def_typescript = []
    need_raw_pinters = False
    return_decl = functions.FUNCTIONS[function].return_type.decl()
    valid_function = return_decl == "const char *" or (not ("*" in return_decl) and not ("[" in return_decl))
    for param in functions.FUNCTIONS[function].parameters:
        param_type = param.type.decl()
        if(param.decltype == "const mjModel *"):
            def_params.append("_model->ptr()")
        elif(param.decltype == "mjData *"):
            def_params.append("_state->ptr()")
        elif(param.decltype == "const char *"):
            def_args  .append("std::string "+param.name)
            def_params.append(param.name+".c_str()")
            def_typescript.append(param.name + " : string")
        elif("mjtNum *" in param.decltype):
            def_args  .append("val "+param.name)
            def_params.append('reinterpret_cast<mjtNum*>('+param.name+'["byteOffset"].as<int>())')
            def_typescript.append(param.name + " : Float64Array")
            need_raw_pinters = True
        elif (not ("*" in param_type) and not ("[" in param_type) and not (param_type == "mjfPluginLibraryLoadCallback")):
            def_args  .append(str(param))
            def_params.append(param.name)
            param_type = param_type.replace("mjtNum","number").replace("int","number").replace("float","number").replace("size_t", "number").replace("unsigned char", "string")
            def_typescript.append(param.name + " : " + param_type)
        else:
            valid_function = False
    if valid_function:
        is_string = False
        to_return = function.ljust(28)+"("+(", ".join(def_params)).ljust(20)
        if return_decl == "const char *":
            return_decl = "std::string"
            to_return = "std::string(" + to_return + ")"
        auto_gen_lines["data_definitions"].append("  "+return_decl.ljust(6)+" "+name.ljust(20)+"("+(", ".join(def_args)).ljust(20)+") { return "+to_return+"); }")
        auto_gen_lines["data_bindings"   ].append('      .function('+('"'+name+'"').ljust(23)+' , &Simulation::'+name.ljust(22)+(')'if not need_raw_pinters else ', allow_raw_pointers())'))
        auto_gen_lines["data_typescript" ].append("  /** "+ functions.FUNCTIONS[function].doc + ("    [Only works with MuJoCo Allocated Arrays!]" if need_raw_pinters else "") +"*/")
        returnType = functions.FUNCTIONS[function].return_type
        returnType = returnType.inner_type.name if "*" in returnType.decl() else returnType.name
        returnType = returnType.replace("mjtNum","number").replace("int","number").replace("float","number").replace("char", "string")
        auto_gen_lines["data_typescript" ].append('  '+name.ljust(22)+'('+", ".join(def_typescript)+'): '+returnType+';')

# Parse mjmodel.h for enums
cur_enum_name = None
for line in model_lines:
    line = line.strip()

    if cur_enum_name is not None and line.startswith("}"):
        cur_enum_name = None
        auto_gen_lines["model_enums"].append('  ;')
        auto_gen_lines["enums_typescript"].append( "}")

    if cur_enum_name is not None and len(line) > 0:
        parts = line.split("//")
        parts = [part.strip() for part in parts]
        if len(parts[0]) > 0 and len(parts[0].split(" ")) > 0:
            meat = parts[0].split(" ")[0].split(",")[0]; potatos = parts[1]
            auto_gen_lines["model_enums"].append('      .value('+('"'+meat+'"').ljust(25)+', '+cur_enum_name.ljust(25)+'::'+meat.ljust(25)+')')
            auto_gen_lines["enums_typescript"].append("    /** "+potatos.ljust(40)+" */")
            auto_gen_lines["enums_typescript"].append("    "+meat.ljust(25)+",")

    if line.startswith("typedef enum"):
        cur_enum_name = line.split(" ")[2][:-1]
        auto_gen_lines["model_enums"].append('  enum_<'+cur_enum_name+'>("'+cur_enum_name+'")')
        if len(line.split("//")) > 1:
            auto_gen_lines["enums_typescript"].append("/** "+line.split("//")[1].ljust(40)+" */")
        auto_gen_lines["enums_typescript"].append("export enum "+cur_enum_name +" {")

def replace_in_file(input_filename, output_filename, replacements):
    try:
        with open(input_filename, 'r') as f:
            content = f.read()

        for placeholder, replacement in replacements.items():
            if placeholder not in content:
                print(f"Warning: Placeholder {placeholder} not found in {input_filename}")
            content = content.replace(placeholder, f"{placeholder}\n{replacement}")

        with open(output_filename, 'w') as f:
            f.write(content)

        print(f"Successfully wrote to {output_filename}")
    except IOError as e:
        print(f"Error: Unable to read or write file: {e}")
        sys.exit(1)

replace_in_file("src/main.template.cc", "src/main.genned.cc", {
    "// MJMODEL_DEFINITIONS": "\n".join(auto_gen_lines["model_definitions"]),
    "// MJMODEL_BINDINGS": "\n".join(auto_gen_lines["model_bindings"]),
    "// MJDATA_DEFINITIONS": "\n".join(auto_gen_lines["data_definitions"]),
    "// MJDATA_BINDINGS": "\n".join(auto_gen_lines["data_bindings"]),
    "// MODEL_ENUMS": "\n".join(auto_gen_lines["model_enums"]),
})

replace_in_file("src/mujoco_wasm.template.d.ts", "dist/mujoco_wasm.d.ts", {
    "// MODEL_INTERFACE": "\n".join(auto_gen_lines["model_typescript"]),
    "// DATA_INTERFACE": "\n".join(auto_gen_lines["data_typescript"]),
    "// ENUMS": "\n".join(auto_gen_lines["enums_typescript"]),
})

# Print debug information
for key, value in auto_gen_lines.items():
    print(f"{key}: {len(value)} lines generated")

