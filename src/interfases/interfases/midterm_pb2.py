# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: midterm.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2
from google.api import annotations_pb2 as google_dot_api_dot_annotations__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\rmidterm.proto\x12\x07Midterm\x1a\x1bgoogle/protobuf/empty.proto\x1a\x1cgoogle/api/annotations.proto\"\"\n\nCoordinate\x12\t\n\x01x\x18\x01 \x01(\x05\x12\t\n\x01y\x18\x02 \x01(\x05\x32g\n\rColorDetector\x12V\n\rgetCoordinate\x12\x16.google.protobuf.Empty\x1a\x13.Midterm.Coordinate\"\x18\x82\xd3\xe4\x93\x02\x12\x12\x10/get_coordinatesB\x0bZ\t./Midtermb\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'midterm_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'Z\t./Midterm'
  _COLORDETECTOR.methods_by_name['getCoordinate']._options = None
  _COLORDETECTOR.methods_by_name['getCoordinate']._serialized_options = b'\202\323\344\223\002\022\022\020/get_coordinates'
  _COORDINATE._serialized_start=85
  _COORDINATE._serialized_end=119
  _COLORDETECTOR._serialized_start=121
  _COLORDETECTOR._serialized_end=224
# @@protoc_insertion_point(module_scope)
