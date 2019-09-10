import argparse
import json
import re

from pymavlink import mavutil


def get_mavlink_enums():
    mavlink_enums = {}
    for ent in mavutil.mavlink.enums:
        enum_dict = {}
        for cmd in mavutil.mavlink.enums[ent]:
            enum = mavutil.mavlink.enums[ent][cmd]
            name = enum.name
            name = name.replace(ent + "_", "")
            if "ENUM_END" in name:
                continue
            enum_dict[cmd] = {
                "name": name,
                "description": enum.description,
                "param": enum.param,
            }
            if enum.param:
                pass
        mavlink_enums[ent] = enum_dict
    return mavlink_enums


def generate_javascript(fn=False):
    json_output = json.dumps(mavlink_enums, indent=2, separators=(",", ": "))
    regex = r'(?<!: )"(\S*?)"'
    javascript_object = re.sub(regex, "\\1", json_output)
    if fn:
        javascript_object = javascript_object.replace("\n", "\n  ")
        output = (
            f"function mavlinkEnumToString (enumValue, enumGroup) {{\n"
            f"  const mavlinkEnums = {javascript_object}\n"
            f"  // Return command text if available\n"
            f"  if (enumGroup in mavlinkEnums) {{\n"
            f"    if (enumValue in mavlinkEnums[enumGroup]) {{\n"
            f"      return mavlinkEnums[enumGroup][enumValue].name\n"
            f"    }}\n"
            f"  }} else {{\n"
            f"    return enumValue\n"
            f"  }}\n"
            f"}}\n"
        )
    else:
        output = javascript_object
    return output


def generate_json():
    json_output = json.dumps(mavlink_enums, indent=4, separators=(",", ": "))
    output = json_output
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-j",
        "--javascript",
        help="output enum as javascript object",
        action="store_true",
    )
    parser.add_argument(
        "-f",
        "--function",
        help="use with -j to wrap enum in a javascript function",
        action="store_true",
    )
    args = parser.parse_args()
    js_output = args.javascript
    js_function = args.function
    output = ""
    filename = ""

    mavlink_enums = get_mavlink_enums()

    if js_output:
        output = generate_javascript(js_function)
        filename = "mavlinkMeta.js"
    else:
        output = generate_json()
        filename = "mavlink_meta.json"

    with open(filename, "w+") as fid:
        fid.write(output)
