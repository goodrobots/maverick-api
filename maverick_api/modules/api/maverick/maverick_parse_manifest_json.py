# test script to parse the output of puppet strings into something
#  that can be used to:
#  - check valid options
#  - provide a schema and autocomplete format for monaco

import json
unknown_types = []

def get_params(puppet_class):
    docstrings = puppet_class.get("docstring", {})
    defaults = puppet_class.get("defaults", {})
    class_properties = {}
    if docstrings:
        for tag in docstrings.get("tags", []):
            tag_properties = {}
            if tag.get("tag_name", "") != "param":
                continue
            property_name_part = tag.get("name", "")
            if not property_name_part:
                continue
            property_name = f"{puppet_class['name']}::{property_name_part}"
            
            tag_docstring = tag.get("text", "")
            base_class_docstring = docstrings.get("text", "")
            tag_properties["description"] = f"{tag_docstring}\n\n{base_class_docstring}"
            
            tag_types = tag.get("types", [])
            if len(tag_types) == 1:
                # Try to parse the type
                tag_type = tag_types[0]
                if tag_type.startswith("Optional["):
                    tag_type = tag_type.lstrip("Optional[").strip()
                    tag_type = tag_type[:-1] # remove the final ']'
                if tag_type == "Any":
                    pass
                elif tag_type == "Boolean":
                    tag_properties["type"] = "boolean"
                elif tag_type == "String":
                    tag_properties["type"] = "string"
                elif tag_type == "Integer":
                    tag_properties["type"] = "number"
                elif tag_type.startswith("Enum"):
                    import ast
                    enum = ast.literal_eval(tag_type.lstrip("Enum"))
                    tag_properties["enum"] = enum
                else:
                    unknown_types.append(tag_type)
                    
            tag_default = defaults.get(property_name_part, None)
            if tag_default is not None:
                tag_type = tag_properties.get("type", None)
                if tag_type == "boolean":
                    tag_default = string_to_bool(tag_default)
                elif tag_type == "number":
                    tag_default = int(tag_default)
                tag_properties["default"] = tag_default

            class_properties[property_name] = tag_properties
    return class_properties

def string_to_bool(v):
    return str(v).lower() in ("yes", "true", "1")


with open("mavdoc.json", "r+") as fid:
    config = json.load(fid)

classes = config["puppet_classes"]
puppet_classes = []
puppet_classes_docs = []
puppet_class_properties = {}
for puppet_class in classes:
    class_name = str(puppet_class.get("name", ""))
    if not class_name:
        # TODO: log error here
        #  the class must have a name
        continue
    if class_name in puppet_classes:
        # TODO: log error here
        #  the class name must be unique
        continue
    if "::" in class_name:
        class_properties = get_params(puppet_class)
        for class_property in class_properties:
            puppet_class_properties[class_property] = class_properties[class_property]
    else:
        class_docs = puppet_class.get("docstring", {})
        puppet_classes_docs.append(str(class_docs.get("text", "")))
        puppet_classes.append(class_name)

monaco_schema = {
    "allowComments": True,
    "type": "object",
    "properties": {
        "classes": {
            "type": "array",
            "description": "puppet classes included in this array are active" ,
            "items": {
                "type": "string",
                "enum": puppet_classes,
                "enumDescriptions": puppet_classes_docs
            }
        }
    },
    "additionalProperties": False,
}
for class_property in puppet_class_properties:
    monaco_schema["properties"][class_property] = puppet_class_properties[class_property]

with open("monaco_schema.json", "w+") as fid:
    json.dump(monaco_schema, fid, indent=4, sort_keys=False)

print(unknown_types)


# {
#     "allowComments": true,
#     type: "object",
#     "properties": {
#         "base::python::maverick_python": {
#             // enum: [true, false],
#             description: 'foo', // docs shown during suggest (not markdown)
#             type: "number",
#             default: 10,
#             title: "title", // this is not markdown
#             markdownDescription: "## this is a test", // docs shown on hover
#         },
#         "base::python::python_version": {
#             enum: ["\"v3.7.6\"", "\"v3.8.1\""],
#             markdownDescription: "## this is also a test",
#             enumDescriptions: ["testing1", "testing2"], // docs shown on hover + during suggest


#         },
#         "classes": {
#         type: "array",
#         description: 'barr',
#         items: {
#             type: "string",
#             enum: ["foo", "barr"]
#         }
#         }
#     },
#     additionalProperties: false,
# }
