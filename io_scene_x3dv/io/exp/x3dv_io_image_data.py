# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 The x3dv-Blender-IO authors.
import re


class ImageData:
    """Contains encoded images"""

    def __init__(self, data: bytes, mime_type: str, name: str):
        self._data = data
        self._mime_type = mime_type
        self._name = name

    def __eq__(self, other):
        return self._data == other.data

    def __hash__(self):
        return hash(self._data)

    def adjusted_name(self):
        regex_dot = re.compile("\.")
        adjusted_name = re.sub(regex_dot, "_", self.name)
        new_name = "".join([char for char in adjusted_name if char not in "!#$&'()*+,/:;<>?@[\]^`{|}~"])
        return new_name

    @property
    def data(self):
        return self._data

    @property
    def name(self):
        return self._name

    @property
    def file_extension(self):
        if self._mime_type == "image/jpeg":
            return ".jpg"
        return ".png"

    @property
    def byte_length(self):
        return len(self._data)
