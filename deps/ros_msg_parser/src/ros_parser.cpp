/****
* © 2022 Carnegie Mellon University. All rights reserved.
* National Robotics Engineering Center, Carnegie Mellon University
* www.nrec.ri.cmu.edu
* Confidential and Proprietary - Do not distribute without prior written permission.
*
* License Status: Not Released.
* (License Status to be confirmed by CTTEC prior to release from NREC)
* This notice must appear in all copies of this file and its derivatives.
****/

/****
* NREC Internal Use (Use as Background IP to be cleared by CTTEC and the Project Manager/PI prior to use on another project).
* Created for Program: HPSTA - 55435.1.1990813
****/

/**** 
* https://github.com/facontidavide/ros_msg_parser/blob/master/src/ros_parser.cpp commit ee66d98
* 
* Original code used under MIT license:
* https://github.com/facontidavide/ros_msg_parser/blob/master/LICENSE 88f1dc0
****/

#include <boost/algorithm/string.hpp>
#include <boost/utility/string_ref.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <functional>
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "ros_msg_parser/ros_parser.hpp"
#include "ros_msg_parser/helper_functions.hpp"

namespace RosMsgParser
{
inline bool operator==(const std::string& a, const boost::string_ref& b)
{
  return (a.size() == b.size() && std::strncmp(a.data(), b.data(), a.size()) == 0);
}

void Parser::registerMessage(const std::string& definition)
{
  const boost::regex msg_separation_regex("^\\s*=+\\n+");

  std::vector<std::string> split;
  std::vector<const ROSType*> all_types;

  boost::split_regex(split, definition, msg_separation_regex);

  _message_info->type_list.reserve(split.size());
  _message_info->type_list.clear();

  for (size_t i = 0; i < split.size(); ++i)
  {
    ROSMessage msg(split[i]);
    if (i == 0)
    {
      msg.mutateType(_msg_type);
    }

    _message_info->type_list.push_back(std::move(msg));
    all_types.push_back(&(_message_info->type_list.back().type()));
  }

  for (ROSMessage& msg : _message_info->type_list)
  {
    msg.updateMissingPkgNames(all_types);
  }
  //------------------------------

  std::function<void(const ROSMessage*, FieldTreeNode*, MessageTreeNode*)> recursiveTreeCreator;

  recursiveTreeCreator = [&](const ROSMessage* msg_definition, FieldTreeNode* field_node, MessageTreeNode* msg_node) {
    // note: should use reserve here, NOT resize
    const size_t NUM_FIELDS = msg_definition->fields().size();

    field_node->children().reserve(NUM_FIELDS);
    msg_node->children().reserve(NUM_FIELDS);

    for (const ROSField& field : msg_definition->fields())
    {
      if (field.isConstant())
      {
        continue;
      }
      // Let's add first a child to string_node
      field_node->addChild(&field);
      FieldTreeNode* new_string_node = &(field_node->children().back());

      const ROSMessage* next_msg = nullptr;
      // builtin types will not trigger a recursion
      if (field.type().isBuiltin() == false)
      {
        next_msg = getMessageByType(field.type());
        if (next_msg == nullptr)
        {
          throw std::runtime_error("This type was not registered ");
        }
        msg_node->addChild(next_msg);
        MessageTreeNode* new_msg_node = &(msg_node->children().back());
        recursiveTreeCreator(next_msg, new_string_node, new_msg_node);

      }  // end of field.isConstant()
    }    // end of for fields
  };     // end of lambda

  auto& first_msg_type = _message_info->type_list.front();
  _message_info->message_tree.root()->setValue(&first_msg_type);
  _message_info->field_tree.root()->setValue( _dummy_root_field.get() );

  // start recursion
  recursiveTreeCreator(&_message_info->type_list.front(), _message_info->field_tree.root(),
                       _message_info->message_tree.root());
}

Parser::Parser(const std::string &topic_name, const ROSType &msg_type, const std::string &definition)
  : _message_info( new ROSMessageInfo)
  , _global_warnings(&std::cerr)
  , _topic_name(topic_name)
  , _msg_type(msg_type)
  , _discard_large_array(DISCARD_LARGE_ARRAYS)
  , _max_array_size(100)
  , _blob_policy(STORE_BLOB_AS_COPY)
  , _dummy_root_field( new ROSField(_msg_type, topic_name) )
{
  registerMessage(definition);
}

const std::shared_ptr<ROSMessageInfo>& Parser::getMessageInfo() const
{
  return _message_info;
}

const ROSMessage* Parser::getMessageByType(const ROSType& type) const
{
  for (const ROSMessage& msg : _message_info->type_list)  // find in the list
  {
    if (msg.type() == type)
    {
      return &msg;
    }
  }
  return nullptr;
}

void Parser::applyVisitorToBuffer(const ROSType& target_type, Span<uint8_t>& buffer,
                                  Parser::VisitingCallback callback) const
{
  if (getMessageByType(target_type) == nullptr)
  {
    // you will not find it. Skip it;
    return;
  }

  std::function<void(const MessageTreeNode*)> recursiveImpl;
  size_t buffer_offset = 0;

  recursiveImpl = [&](const MessageTreeNode* msg_node) {
    const ROSMessage* msg_definition = msg_node->value();
    const ROSType& msg_type = msg_definition->type();

    const bool matching = (msg_type == target_type);

    uint8_t* prev_buffer_ptr = buffer.data() + buffer_offset;
    size_t prev_offset = buffer_offset;

    size_t index_m = 0;

    for (const ROSField& field : msg_definition->fields())
    {
      if (field.isConstant())
        continue;

      const ROSType& field_type = field.type();

      int32_t array_size = field.arraySize();
      if (array_size == -1)
      {
        ReadFromBuffer(buffer, buffer_offset, array_size);
      }

      //------------------------------------

      if (field_type.isBuiltin())
      {
        for (int i = 0; i < array_size; i++)
        {
          // Skip
          ReadFromBufferToVariant(field_type.typeID(), buffer, buffer_offset);
        }
      }
      else
      {
        // field_type.typeID() == OTHER
        for (int i = 0; i < array_size; i++)
        {
          recursiveImpl(msg_node->child(index_m));
        }
        index_m++;
      }
    }  // end for fields
    if (matching)
    {
      Span<uint8_t> view(prev_buffer_ptr, buffer_offset - prev_offset);
      callback(msg_type, view);
    }
  };  // end lambda

  // start recursion
  recursiveImpl(_message_info->message_tree.croot());
}

template <typename Container>
inline void ExpandVectorIfNecessary(Container& container, size_t new_size)
{
  if (container.size() <= new_size)
  {
    const size_t increased_size = std::max(size_t(32), container.size() * 2);
    container.resize(increased_size);
  }
}

bool Parser::deserializeIntoFlatMsg(Span<const uint8_t> buffer, FlatMessage* flat_container) const
{
  bool entire_message_parse = true;

  size_t value_index = 0;
  size_t name_index = 0;
  size_t blob_index = 0;
  size_t blob_storage_index = 0;

  size_t buffer_offset = 0;

  std::function<void(const MessageTreeNode*, FieldTreeLeaf, bool)> deserializeImpl;

  deserializeImpl = [&](const MessageTreeNode* msg_node, FieldTreeLeaf tree_leaf, bool store) {
    const ROSMessage* msg_definition = msg_node->value();
    size_t index_s = 0;
    size_t index_m = 0;

    for (const ROSField& field : msg_definition->fields())
    {
      bool DO_STORE = store;
      if (field.isConstant())
        continue;

      const ROSType& field_type = field.type();

      auto new_tree_leaf = tree_leaf;
      new_tree_leaf.node_ptr = tree_leaf.node_ptr->child(index_s);

      int32_t array_size = field.arraySize();
      if (array_size == -1)
      {
        ReadFromBuffer(buffer, buffer_offset, array_size);
      }
      if (field.isArray())
      {
        new_tree_leaf.index_array.push_back(0);
      }

      bool IS_BLOB = false;


      // Stop storing it if is NOT a blob and a very large array.
      if (array_size > static_cast<int32_t>(_max_array_size) &&
          (field_type.typeID() == BuiltinType::OTHER))
      {
        
          if (_discard_large_array)
          {
            DO_STORE = false;
          }
          entire_message_parse = false;
      }

      // process all non-string primitives to be parsed as a blob by reference 
      if (builtinSize(field_type.typeID()) != -1)  
      {
        ExpandVectorIfNecessary(flat_container->blob, blob_index);

        if (buffer_offset + array_size > static_cast<std::size_t>(buffer.size()))
        {
          throw std::runtime_error("Buffer overrun in deserializeIntoFlatContainer (blob)");
        }
        if (DO_STORE)
        {
          flat_container->blob[blob_index].first = new_tree_leaf;
          auto& blob = flat_container->blob[blob_index].second;
          blob_index++;

          if (_blob_policy == STORE_BLOB_AS_COPY)
          {
            ExpandVectorIfNecessary(flat_container->blob_storage, blob_storage_index);

            auto& storage = flat_container->blob_storage[blob_storage_index];
            storage.resize(array_size);
            std::memcpy(storage.data(), &buffer[buffer_offset], array_size);
            blob_storage_index++;

            blob = Span<const uint8_t>(storage.data(), storage.size());
          }
          else
          {
            blob = Span<const uint8_t>(&buffer[buffer_offset], array_size * builtinSize(field_type.typeID()));
          }
        }
        buffer_offset += array_size * builtinSize(field_type.typeID());

      } 
      // if processing a string and it should be stored by reference
      else if (field_type.typeID() == STRING && !STORE_BLOB_AS_COPY)
      {
        ExpandVectorIfNecessary(flat_container->blob, blob_index);

        // read string size, which is recorded in first 4 bytes of string data
        // Note: ReadFromBuffer increments buffer by sizeof(third argument)
        uint32_t string_size = 0;
        ReadFromBuffer(buffer, buffer_offset, string_size);

        // calculate total data size of the string (chars + size data)
        uint32_t span_size = string_size + sizeof(string_size);

        // decrement buffer pointer since ReadFromBuffer incremented over the size data
        buffer_offset -= sizeof(string_size);

        if (buffer_offset + string_size > static_cast<std::size_t>(buffer.size()))
        {
          throw std::runtime_error("Buffer overrun in deserializeIntoFlatContainer (blob)");
        }

        if (DO_STORE)
        {
          // create span over existing string data w/ size info
          auto& blob = flat_container->blob[blob_index].second;

          blob = Span<const uint8_t>(&buffer[buffer_offset], span_size);
          flat_container->blob[blob_index].first = new_tree_leaf;

          blob_index++;

        }
        buffer_offset += span_size;

      }
      else  // type is OTHER or string to be copied
      {
        bool DO_STORE_ARRAY = DO_STORE;
        for (int i = 0; i < array_size; i++)
        {
          if (DO_STORE_ARRAY && i >= static_cast<int32_t>(_max_array_size))
          {
            DO_STORE_ARRAY = false;
          }

          if (field.isArray() && DO_STORE_ARRAY)
          {
            new_tree_leaf.index_array.back() = i;
          }

          if (field_type.typeID() == STRING)
          {
            ExpandVectorIfNecessary(flat_container->name, name_index);

            uint32_t string_size = 0;
            ReadFromBuffer(buffer, buffer_offset, string_size);

            if (buffer_offset + string_size > static_cast<std::size_t>(buffer.size()))
            {
              throw std::runtime_error("Buffer overrun in RosMsgParser::ReadFromBuffer");
            }

            if (DO_STORE_ARRAY)
            {
              if (string_size == 0)
              {
                // corner case, when there is an empty string at the end of the message
                flat_container->name[name_index].second.clear();
              }
              else
              {
                const char* buffer_ptr = reinterpret_cast<const char*>(buffer.data() + buffer_offset);
                flat_container->name[name_index].second.assign(buffer_ptr, string_size);
              }
              flat_container->name[name_index].first = new_tree_leaf;
              name_index++;
            }
            buffer_offset += string_size;
          }
          else
          {  // field_type.typeID() == OTHER

            deserializeImpl(msg_node->child(index_m), new_tree_leaf, DO_STORE_ARRAY);
          }
        }  // end for array_size
      }

      if (field_type.typeID() == OTHER)
      {
        index_m++;
      }
      index_s++;
    }  // end for fields
  };   // end of lambda

  // pass the shared_ptr
  flat_container->msg_info = _message_info;

  FieldTreeLeaf rootnode;
  rootnode.node_ptr = _message_info->field_tree.croot();

  deserializeImpl(_message_info->message_tree.croot(), rootnode, true);

  flat_container->name.resize(name_index);
  flat_container->value.resize(value_index);
  flat_container->blob.resize(blob_index);
  flat_container->blob_storage.resize(blob_storage_index);

  if (buffer_offset != static_cast<std::size_t>(buffer.size()))
  {
    char msg_buff[1000];
    sprintf(msg_buff,
            "buildRosFlatType: There was an error parsing the buffer.\n"
            "Size %d != %d, while parsing [%s]",
            (int)buffer_offset, (int)buffer.size(), _topic_name.c_str());

    throw std::runtime_error(msg_buff);
  }
  return entire_message_parse;
}

bool Parser::deserializeIntoJson(Span<const uint8_t> buffer, std::string* json_txt, bool pretty_printer) const
{
  rapidjson::Document json_document;
  rapidjson::Document::AllocatorType& alloc = json_document.GetAllocator();

  size_t buffer_offset = 0;

  std::function<void(const MessageTreeNode*, rapidjson::Value&)> deserializeImpl;

  deserializeImpl = [&](const MessageTreeNode* msg_node, rapidjson::Value& json_value) {
    const ROSMessage* msg_definition = msg_node->value();
    size_t index_s = 0;
    size_t index_m = 0;

    for (const ROSField& field : msg_definition->fields())
    {
      if (field.isConstant())
        continue;

      const ROSType& field_type = field.type();
      auto field_name = rapidjson::StringRef(field.name().data(), field.name().size());

      int32_t array_size = field.arraySize();
      if (array_size == -1)
      {
        ReadFromBuffer(buffer, buffer_offset, array_size);
      }

      // Stop storing if it is a blob.
      if (array_size > static_cast<int32_t>(_max_array_size))
      {
        if (buffer_offset + array_size > static_cast<std::size_t>(buffer.size()))
        {
          throw std::runtime_error("Buffer overrun in blob");
        }
        buffer_offset += array_size;
      }
      else  // NOT a BLOB
      {
        rapidjson::Value array_value(rapidjson::kArrayType);

        for (int i = 0; i < array_size; i++)
        {
          rapidjson::Value new_value;
          new_value.SetObject();

          switch (field_type.typeID())
          {
            case BOOL:
              new_value.SetBool(ReadFromBuffer<bool>(buffer, buffer_offset));
              break;
            case CHAR:
            {
              char c = ReadFromBuffer<char>(buffer, buffer_offset);
              new_value.SetString(&c, 1, alloc);
            }
            break;
            case BYTE:
            case UINT8:
              new_value.SetUint(ReadFromBuffer<uint8_t>(buffer, buffer_offset));
              break;
            case UINT16:
              new_value.SetUint(ReadFromBuffer<uint16_t>(buffer, buffer_offset));
              break;
            case UINT32:
              new_value.SetUint(ReadFromBuffer<uint32_t>(buffer, buffer_offset));
              break;
            case UINT64:
              new_value.SetUint64(ReadFromBuffer<uint64_t>(buffer, buffer_offset));
              break;
            case INT8:
              new_value.SetInt(ReadFromBuffer<int8_t>(buffer, buffer_offset));
              break;
            case INT16:
              new_value.SetInt(ReadFromBuffer<int16_t>(buffer, buffer_offset));
              break;
            case INT32:
              new_value.SetInt(ReadFromBuffer<int32_t>(buffer, buffer_offset));
              break;
            case INT64:
              new_value.SetInt64(ReadFromBuffer<int64_t>(buffer, buffer_offset));
              break;
            case FLOAT32:
              new_value.SetFloat(ReadFromBuffer<float>(buffer, buffer_offset));
              break;
            case FLOAT64:
              new_value.SetDouble(ReadFromBuffer<double>(buffer, buffer_offset));
              break;
            case TIME:
            {
              ros::Time tmp;
              ReadFromBuffer(buffer, buffer_offset, tmp.sec);
              ReadFromBuffer(buffer, buffer_offset, tmp.nsec);
              new_value.SetDouble(tmp.toSec());
            }
            break;
            case DURATION:
            {
              ros::Duration tmp;
              ReadFromBuffer(buffer, buffer_offset, tmp.sec);
              ReadFromBuffer(buffer, buffer_offset, tmp.nsec);
              new_value.SetDouble(tmp.toSec());
            }
            break;

            case STRING:
            {
              uint32_t string_size = 0;
              ReadFromBuffer(buffer, buffer_offset, string_size);
              if (buffer_offset + string_size > static_cast<std::size_t>(buffer.size()))
              {
                throw std::runtime_error("Buffer overrun");
              }
              new_value.SetString(reinterpret_cast<const char*>(&buffer[buffer_offset]), string_size, alloc);
              buffer_offset += string_size;
            }
            break;
            case OTHER:
            {
              deserializeImpl(msg_node->child(index_m), new_value);
            }
            break;
          }  // end switch

          if (field.isArray())
          {
            array_value.PushBack(new_value, alloc);
          }
          else
          {
            json_value.AddMember(field_name, new_value, alloc);
          }
        }  // end for array

        if (field.isArray())
        {
          json_value.AddMember(field_name, array_value, alloc);
        }
      }  // end for array_size

      if (field_type.typeID() == OTHER)
      {
        index_m++;
      }
      index_s++;
    }  // end for fields
  };   // end of lambda

  // pass the shared_ptr

  FieldTreeLeaf rootnode;
  rootnode.node_ptr = _message_info->field_tree.croot();

  json_document.SetObject();
  rapidjson::Value json_node;
  json_node.SetObject();

  deserializeImpl(_message_info->message_tree.croot(), json_node);

  auto topic_name = rapidjson::StringRef(_topic_name.data(), _topic_name.size());
  json_document.AddMember("topic", topic_name, alloc);
  json_document.AddMember("msg", json_node, alloc);

  rapidjson::StringBuffer json_buffer;
  json_buffer.Reserve(2048);

  if( pretty_printer ){
    rapidjson::PrettyWriter<rapidjson::StringBuffer,
                            rapidjson::UTF8<>,
                            rapidjson::UTF8<>,
                            rapidjson::CrtAllocator,
                            rapidjson::kWriteDefaultFlags |
                              rapidjson::kWriteNanAndInfFlag> json_writer(json_buffer);
    json_document.Accept(json_writer);
  }
  else{
    rapidjson::Writer<rapidjson::StringBuffer,
                      rapidjson::UTF8<>,
                      rapidjson::UTF8<>,
                      rapidjson::CrtAllocator,
                      rapidjson::kWriteDefaultFlags |
                        rapidjson::kWriteNanAndInfFlag> json_writer(json_buffer);
    json_document.Accept(json_writer);
  }
  *json_txt = json_buffer.GetString();

  return true;
}

void CreateRenamedValues(const FlatMessage& flat_msg, RenamedValues& renamed)
{
  renamed.resize(flat_msg.value.size());
  for (size_t i = 0; i < flat_msg.value.size(); i++)
  {
    const auto& in = flat_msg.value[i];
    auto& out = renamed[i];
    in.first.toStr(out.first);
    out.second = in.second.convert<double>();
  }
}

void ParsersCollection::registerParser(const std::string& topic_name, const ROSType& msg_type,
                                       const std::string& definition)
{
  auto it = _pack.find(topic_name);
  if (it == _pack.end())
  {
    Parser parser(topic_name, msg_type, definition);
    CachedPack pack = { std::move(parser), {} };
    _pack.insert({ topic_name, std::move(pack) });
  }
}

void ParsersCollection::registerParser(const std::string& topic_name, const ShapeShifter& msg)
{
  registerParser(topic_name, msg.getDataType(), msg.getMessageDefinition());
}

void ParsersCollection::registerParser(const std::string& topic_name, const rosbag::ConnectionInfo& connection)
{
  registerParser(topic_name, connection.datatype, connection.msg_def);
}

const Parser* ParsersCollection::getParser(const std::string& topic_name) const
{
  auto it = _pack.find(topic_name);
  if (it != _pack.end())
  {
    return &it->second.parser;
  }
  return nullptr;
}

const ParsersCollection::DeserializedMsg* ParsersCollection::deserialize(const std::string& topic_name,
                                                                         Span<const uint8_t> buffer)
{
  auto it = _pack.find(topic_name);
  if (it != _pack.end())
  {
    CachedPack& pack = it->second;
    Parser& parser = pack.parser;
    FlatMessage& flat_msg = pack.msg.flat_msg;
    RenamedValues& renamed = pack.msg.renamed_vals;

    parser.deserializeIntoFlatMsg(buffer, &flat_msg);
    CreateRenamedValues(flat_msg, renamed);

    return &pack.msg;
  }
  return nullptr;
}

const ParsersCollection::DeserializedMsg* ParsersCollection::deserialize(const std::string& topic_name,
                                                                         const ShapeShifter& msg)
{
  Span<const uint8_t> buffer(msg.raw_data(), msg.size());
  return deserialize(topic_name, buffer);
}

const ParsersCollection::DeserializedMsg* ParsersCollection::deserialize(const std::string& topic_name,
                                                                         const rosbag::MessageInstance& msg)
{
  auto it = _pack.find(topic_name);
  if (it != _pack.end())
  {
    CachedPack& pack = it->second;
    Parser& parser = pack.parser;
    FlatMessage& flat_msg = pack.msg.flat_msg;
    RenamedValues& renamed = pack.msg.renamed_vals;

    // write the message into the buffer
    _buffer.resize(msg.size());
    ros::serialization::OStream stream(_buffer.data(), msg.size());
    msg.write(stream);

    Span<const uint8_t> buffer(_buffer.data(), msg.size());

    parser.deserializeIntoFlatMsg(buffer, &flat_msg);
    CreateRenamedValues(flat_msg, renamed);

    return &pack.msg;
  }
  return nullptr;
}

}  // namespace RosMsgParser
