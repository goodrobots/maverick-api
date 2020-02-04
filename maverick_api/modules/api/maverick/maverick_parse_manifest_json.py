# test script to parse the output of puppet strings into something
#  that can be used to:
#  - check valid options
#  - provide a schema and autocomplete format for monaco

import json
import ast


def get_params(puppet_class, unknown_types):
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
                    tag_type = tag_type[:-1]  # remove the final ']'
                if tag_type == "Any":
                    pass
                elif tag_type == "Boolean":
                    tag_properties["type"] = "boolean"
                elif tag_type.lower() == "string":
                    tag_properties["type"] = "string"
                elif tag_type == "Integer":
                    tag_properties["type"] = "number"
                    tag_properties["_type"] = "integer"
                elif tag_type == "Float":
                    tag_properties["type"] = "number"
                    tag_properties["_type"] = "float"
                elif tag_type.startswith("Enum"):

                    enum = ast.literal_eval(tag_type.lstrip("Enum"))
                    tag_properties["enum"] = enum
                elif tag_type == "Array[String]":
                    tag_properties["type"] = "array"
                    tag_properties["items"] = {"type": "string"}
                else:
                    unknown_types.append(
                        {
                            "type": tag_type,
                            "property_name": property_name,
                            "defaults": defaults.get(property_name_part, None),
                        }
                    )
            elif tag_types:
                unknown_types.append(
                    {
                        "type": tag_type,
                        "property_name": property_name,
                        "defaults": defaults.get(property_name_part, None),
                    }
                )

            tag_default = defaults.get(property_name_part, None)
            if tag_default is not None:
                if tag_default.startswith('"') or tag_default.startswith("'"):
                    tag_default = tag_default[1:]
                if tag_default.endswith('"') or tag_default.endswith("'"):
                    tag_default = tag_default[:-1]

                tag_type = tag_properties.get("type", None)

                if tag_default in ("undef"):
                    # don't attempt to set defaults for undefined
                    continue
                elif tag_default.startswith("{") and tag_default.endswith("}"):
                    # don't attempt to set defaults for hashes
                    continue
                elif tag_type == "boolean":
                    tag_default = string_to_bool(tag_default)
                elif tag_type == "number":
                    if tag_properties["_type"] == "integer":
                        tag_default = int(tag_default)
                    elif tag_properties["_type"] == "float":
                        tag_default = float(tag_default)
                elif tag_type is None and tag_default in ("true", "false"):
                    tag_default = string_to_bool(tag_default)
                    tag_properties["type"] = "boolean"
                elif tag_type == "array":
                    array_types = tag_properties["items"]["type"]
                    tag_default = ast.literal_eval(tag_default)
                    if array_types == "string":
                        tag_default = [str(x) for x in tag_default]
                tag_properties["default"] = tag_default
            class_properties[property_name] = tag_properties
    return class_properties, unknown_types


def string_to_bool(v):
    v = v.lower()
    if v in ("true", "false"):
        return v in ("yes")
    else:
        raise ValueError


def parse_puppet_strings(input_filepath, output_filepath, debug=False):
    unknown_types = []
    with open(input_filepath, "r+") as fid:
        config = json.load(fid)

    classes = config["puppet_classes"]
    puppet_classes = []
    puppet_classes_docs = []
    puppet_class_properties = {}
    for puppet_class in classes:
        class_name = puppet_class.get("name", "")
        if not class_name:
            # TODO: log error here
            #  the class must have a name
            continue
        if class_name in puppet_classes:
            # TODO: log error here
            #  the class name must be unique
            continue

        class_docs = puppet_class.get("docstring", {})
        tags = class_docs.get("tags", [])
        class_docs_summary = ""
        for tag in tags:
            if tag.get("tag_name", "").lower() == "summary":
                class_docs_summary = tag.get("text", "")
                break
        puppet_classes_docs.append(class_docs_summary)
        puppet_classes.append(class_name)

        class_properties, unknown_types = get_params(puppet_class, unknown_types)
        for class_property in class_properties:
            puppet_class_properties[class_property] = class_properties[class_property]

    defined_types = config["defined_types"]
    for defined_type in defined_types:
        type_properties, unknown_types = get_params(defined_type, unknown_types)
        for type_property in type_properties:
            puppet_class_properties[type_property] = type_properties[type_property]

    monaco_schema = {
        "allowComments": True,
        "type": "object",
        "properties": {
            "classes": {
                "type": "array",
                "description": "puppet classes included in this array are active",
                "items": {
                    "type": "string",
                    "enum": puppet_classes,
                    "enumDescriptions": puppet_classes_docs,
                },
            }
        },
        "additionalProperties": False,
    }
    for class_property in puppet_class_properties:
        monaco_schema["properties"][class_property] = puppet_class_properties[
            class_property
        ]

    with open(output_filepath, "w+") as fid:
        json.dump(monaco_schema, fid, indent=4, sort_keys=False)

    if debug:
        print(unknown_types)


if __name__ == "__main__":
    input_filepath = "mavdoc.json"
    output_filepath = "monaco_schema.json"
    parse_puppet_strings(input_filepath, output_filepath, debug=True)

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
