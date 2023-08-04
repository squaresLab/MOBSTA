// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosbag2_interfaces:msg/ReadSplitEvent.idl
// generated code does not contain a copyright notice
#include "rosbag2_interfaces/msg/detail/read_split_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `closed_file`
// Member `opened_file`
#include "rosidl_runtime_c/string_functions.h"

bool
rosbag2_interfaces__msg__ReadSplitEvent__init(rosbag2_interfaces__msg__ReadSplitEvent * msg)
{
  if (!msg) {
    return false;
  }
  // closed_file
  if (!rosidl_runtime_c__String__init(&msg->closed_file)) {
    rosbag2_interfaces__msg__ReadSplitEvent__fini(msg);
    return false;
  }
  // opened_file
  if (!rosidl_runtime_c__String__init(&msg->opened_file)) {
    rosbag2_interfaces__msg__ReadSplitEvent__fini(msg);
    return false;
  }
  return true;
}

void
rosbag2_interfaces__msg__ReadSplitEvent__fini(rosbag2_interfaces__msg__ReadSplitEvent * msg)
{
  if (!msg) {
    return;
  }
  // closed_file
  rosidl_runtime_c__String__fini(&msg->closed_file);
  // opened_file
  rosidl_runtime_c__String__fini(&msg->opened_file);
}

bool
rosbag2_interfaces__msg__ReadSplitEvent__are_equal(const rosbag2_interfaces__msg__ReadSplitEvent * lhs, const rosbag2_interfaces__msg__ReadSplitEvent * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // closed_file
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->closed_file), &(rhs->closed_file)))
  {
    return false;
  }
  // opened_file
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->opened_file), &(rhs->opened_file)))
  {
    return false;
  }
  return true;
}

bool
rosbag2_interfaces__msg__ReadSplitEvent__copy(
  const rosbag2_interfaces__msg__ReadSplitEvent * input,
  rosbag2_interfaces__msg__ReadSplitEvent * output)
{
  if (!input || !output) {
    return false;
  }
  // closed_file
  if (!rosidl_runtime_c__String__copy(
      &(input->closed_file), &(output->closed_file)))
  {
    return false;
  }
  // opened_file
  if (!rosidl_runtime_c__String__copy(
      &(input->opened_file), &(output->opened_file)))
  {
    return false;
  }
  return true;
}

rosbag2_interfaces__msg__ReadSplitEvent *
rosbag2_interfaces__msg__ReadSplitEvent__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbag2_interfaces__msg__ReadSplitEvent * msg = (rosbag2_interfaces__msg__ReadSplitEvent *)allocator.allocate(sizeof(rosbag2_interfaces__msg__ReadSplitEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosbag2_interfaces__msg__ReadSplitEvent));
  bool success = rosbag2_interfaces__msg__ReadSplitEvent__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosbag2_interfaces__msg__ReadSplitEvent__destroy(rosbag2_interfaces__msg__ReadSplitEvent * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosbag2_interfaces__msg__ReadSplitEvent__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosbag2_interfaces__msg__ReadSplitEvent__Sequence__init(rosbag2_interfaces__msg__ReadSplitEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbag2_interfaces__msg__ReadSplitEvent * data = NULL;

  if (size) {
    data = (rosbag2_interfaces__msg__ReadSplitEvent *)allocator.zero_allocate(size, sizeof(rosbag2_interfaces__msg__ReadSplitEvent), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosbag2_interfaces__msg__ReadSplitEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosbag2_interfaces__msg__ReadSplitEvent__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rosbag2_interfaces__msg__ReadSplitEvent__Sequence__fini(rosbag2_interfaces__msg__ReadSplitEvent__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rosbag2_interfaces__msg__ReadSplitEvent__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rosbag2_interfaces__msg__ReadSplitEvent__Sequence *
rosbag2_interfaces__msg__ReadSplitEvent__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosbag2_interfaces__msg__ReadSplitEvent__Sequence * array = (rosbag2_interfaces__msg__ReadSplitEvent__Sequence *)allocator.allocate(sizeof(rosbag2_interfaces__msg__ReadSplitEvent__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosbag2_interfaces__msg__ReadSplitEvent__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosbag2_interfaces__msg__ReadSplitEvent__Sequence__destroy(rosbag2_interfaces__msg__ReadSplitEvent__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosbag2_interfaces__msg__ReadSplitEvent__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosbag2_interfaces__msg__ReadSplitEvent__Sequence__are_equal(const rosbag2_interfaces__msg__ReadSplitEvent__Sequence * lhs, const rosbag2_interfaces__msg__ReadSplitEvent__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosbag2_interfaces__msg__ReadSplitEvent__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosbag2_interfaces__msg__ReadSplitEvent__Sequence__copy(
  const rosbag2_interfaces__msg__ReadSplitEvent__Sequence * input,
  rosbag2_interfaces__msg__ReadSplitEvent__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosbag2_interfaces__msg__ReadSplitEvent);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosbag2_interfaces__msg__ReadSplitEvent * data =
      (rosbag2_interfaces__msg__ReadSplitEvent *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosbag2_interfaces__msg__ReadSplitEvent__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosbag2_interfaces__msg__ReadSplitEvent__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosbag2_interfaces__msg__ReadSplitEvent__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
