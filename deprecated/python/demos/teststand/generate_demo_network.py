# Copyright 2019 Max Planck Society. All rights reserved.

import os
import os.path
import shutil

import tensorflow as tf
from tensorflow.python.framework import ops
from tensorflow.python.ops import variables
from tensorflow.python.saved_model import builder as saved_model_builder
from tensorflow.python.saved_model import loader
from tensorflow.python.client import session


# This is based on
#    https://medium.com/jim-fleming/loading-a-tensorflow-graph-with-the-c-api-4caaff88463f
#
#  For the saver work
#    https://github.com/tensorflow/tensorflow/blob/2eac53d5ea540b0b09326ba9330b6051d742532d/tensorflow/python/saved_model/saved_model_test.py#L155


if __name__ == "__main__":
    export_dir = "policy_dir"
    if os.path.exists(export_dir):
        shutil.rmtree(export_dir)

    builder = saved_model_builder.SavedModelBuilder(export_dir)

    # This is based on
    #   https://medium.com/jim-fleming/loading-a-tensorflow-graph-with-the-c-api-4caaff88463f
    graph = tf.Graph()
    sess = tf.Session(graph=graph)

    with graph.as_default():
        # Example of variable input length.
        x_in = tf.placeholder(
            shape=(None, 42), name="input/Ob", dtype=tf.float32
        )
        action = tf.constant(
            [[0.5, -1.0]], name="output/action", dtype=tf.float32
        )

        sess.run(tf.global_variables_initializer())

    # For demonstration purpose - could also run the following export line
    # inside the `with sess` block.
    #
    # NOTE: Need to set the default graph. Otherwise this will export an
    #       empty graph.
    with sess.graph.as_default():
        builder.add_meta_graph_and_variables(sess, ["EVALUATION"])

    # Save the SavedModel to disk.
    builder.save(as_text=False)

    print ""
    print 'Saved an example tf-model in "{}"'.format(export_dir)
    print ""

    # Verify the model loads in python.
    sess = session.Session(graph=ops.Graph())
    loader.load(sess, ["EVALUATION"], export_dir)
