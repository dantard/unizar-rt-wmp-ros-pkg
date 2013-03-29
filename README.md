unizar-rt-wmp-ros-pkg
=====================

The ros_rt_wmp is a ROS node capable of replicating whatever ROS topic or service in another computer wirelessly connected with the source without the need of sharing the same roscore.

As an example consider a team of robots building cooperatively a map. The robots have to exchange their laser and pose information. However if the network that connect the robots is not completely connected and they can't use an infrastructure network there is no way to share data among robots using the ROS unless a routing protocol is set up on top of the IP protocol.

The ros_rt_wmp nodes allow that, creating a multi-hop firm real-time network.

* Example

Suppose that robot R0 has to share its \laser data with R4 that is, from the point of view of the network topology, 5 hops away and '''can't/don't want to''' share the same roscore. To do that is sufficient to instanciate two ros_rt_wmp nodes, the first on R0 and the second on R4. The \laser topic on R0 is remapped to the ros_rt_wmp "input" topic called (for example) /R0/tx/laser. It will provoke the node to send the data published in that topic to node R4 through the network.

At the destination node a topic /R4/rx/R0/laser that replicates the /laser topic on R0 will appear automatically. It means that on R4 we are receiving data from R0 of type laser.

The communication can be bidirectional if we follow the same steps on node R4. It is possible to use whatever number of topics and of whatever type. It is possible to do a similar thing with service also. If we have a service at R1 called /factorial, and we want to call it from R5, we can configure the ros_rt_wmp nodes to have a service /R4/R0/factorial on R4 that, if called, will transparently call the service /factorial on R0 and will return the result to the caller.

To summarize, ros_rt_wmp allows to distribute/decentralize a complex robotics system in multiple computation unit in a transparent form: the only requisite is to know which data from other robots we need in each one of them.

* Under the hood

The ros_rt_wmp nodes uses the RT-WMP protocol [1-5] that allows real time communication in wireless mobile ad-hoc networks and supports multi-hop and message priorities.

The data published in the topics are packed by the source ros_rt_wmp node in RT-WMP messages and delivered to the destination node. The latter unpack them and publish data to the correspondent topic. A similar (but two-way) process takes place for services. Since the RT-WMP is capable of managing efficiently the mobility of the nodes, the process is completely transparent to the users that have only to configure the needed topic/services and source/destination for each one of them.

* References

[1] D. Tardioli, '''Real-Time Communication in Wireless ad-hoc networks. The RT-WMP protocol''', PhD Thesis, Universidad de Zaragoza, October, 2010

[2] D. Tardioli, J. L. Villarroel, '''Routing Wireless Real-Time Traffic Using Minimum Spanning Trees''', International Conference on Computing, Networking and Communications, To appear, 2012

[3] D. Tardioli, L. Almeida, J. L. Villarroel, '''Adding Alien Traffic Endurance to Wireless Token-Passing Real-Time Protocols''', In the 2010 Asia-Pacific Services Computing Conference, 2010

[4] D. Sicignano, D. Tardioli and J. L. Villarroel, '''QoS over Real Time wireless multihop Network''', In the First International Conference on Ad-hoc Networks, 2009

[5] D. Tardioli and J. L. Villarroel, '''Real-Time Communications over 802.11: RT-WMP''', In the fourth IEEE international Conference on Mobile Ad-hoc and Sensor Systems, 2007
