"""
@ 2023 Carnegie Mellon University. All rights reserved.
National Robotics Engineering Center, Carnegie Mellon University
www.nrec.ri.cmu.edu
Confidential and Proprietary - Do not distribute without prior written
permission.

License Status: Not Released.
(License Status to be confirmed by CTTEC prior to release from NREC)
This notice must appear in all copies of this file and its derivatives.
"""

"""
NREC Internal Use (Use as Background IP to be cleared by CTTEC and the Project
Manager/PI prior to use on another project).
Created for Program: HPSTA - 55435.1.1990813
"""

# https://github.com/ros2/rosbag2/blob/0.15.3/rosbag2_py/rosbag2_py/__init__.py commit d6ffd91
# Original code used under Apache license:
#  Software License Agreement (Apache License)

# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rpyutils import add_dll_directories_from_env

# Since Python 3.8, on Windows we should ensure DLL directories are explicitly added
# to the search path.
# See https://docs.python.org/3/whatsnew/3.8.html#bpo-36085-whatsnew
with add_dll_directories_from_env('PATH'):
    from rosbag2_py._reader import (
        SequentialCompressionReader,
        SequentialReader,
        get_registered_readers,
    )
    from rosbag2_py._storage import (
        ConverterOptions,
        StorageFilter,
        StorageOptions,
        TopicMetadata,
        TopicInformation,
        BagMetadata,
    )
    from rosbag2_py._writer import (
        SequentialCompressionWriter,
        SequentialWriter,
        get_registered_writers,
        get_registered_compressors,
        get_registered_serializers,
    )
    from rosbag2_py._info import (
        Info,
    )
    from rosbag2_py._transport import (
        bag_rewrite,
        Player,
        MutationPlayer,
        PlayOptions,
        Recorder,
        RecordOptions,
    )
    from rosbag2_py._reindexer import (
        Reindexer
    )

__all__ = [
    'bag_rewrite',
    'ConverterOptions',
    'get_registered_readers',
    'get_registered_writers',
    'get_registered_compressors',
    'get_registered_serializers',
    'Reindexer',
    'SequentialCompressionReader',
    'SequentialCompressionWriter',
    'SequentialReader',
    'SequentialWriter',
    'StorageFilter',
    'StorageOptions',
    'TopicMetadata',
    'TopicInformation',
    'BagMetadata',
    'Info',
    'Player',
    'MutationPlayer',
    'PlayOptions',
    'Recorder',
    'RecordOptions',
]
