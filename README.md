#MOLAR

Multi Object Learning Architecture for Robots

This is the MOLAR package. It is a rough, bare-bones set of python classes and associated ROS services for implementing a discrete-view perception pipeline for an autonomous, learning robot. It is divided into several conceptual layers, with each layer being an independent ROS node responsible for a certain peice of perception functionality. Communication between layers is achieved via appropriate services and ROS messages exchanged between nodes.

We expect the use-case of MOLAR to be as follows: A developer clones the repository, and uses the provided python classes to write wrappers around their own algorithms (such as scene segmentation, data writing, filtering etc.), these then communicate through the provided services and messages, with MOLAR handling the organisation and overall flow of information in the system.
