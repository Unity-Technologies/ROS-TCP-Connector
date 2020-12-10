# ROSGeometry component

In Unity, the X axis points right, Y up, and Z forward. ROS, on the other hand, supports various coordinate frames: in the most commonly-used one, X points forward, Y left, and Z up. In ROS terminology, this frame is called "FLU" (forward, left, up), whereas the Unity coordinate frame would be "RUF" (right, up, forward).

The ROSGeometry namespace contains code to make it easier to work with these various coordinate frames - letting you be explicit about what coordinates a given value is in at compile time, and managing the conversions for you. It does this with two generic structs, `Vector3<C>` and `Quaternion<C>`. The type parameter C indicates the coordinate frame you're working in - either FLU, or RUF, or perhaps a more exotic frame such as NED (north, east, down) or ENU (east, north, up), commonly used in aviation.


# Converting between frames:

For example, if you need to convert an object's position into the FLU coordinate frame, you might write:

    Vector3<FLU> rosPos = obj.transform.position.To<FLU>();
   
An explicit cast, or calling the constructor, would also produce the same result.

    Vector3<FLU> rosPos2 = (Vector3<FLU>)obj.transform.position;
    Vector3<FLU> rosPos3 = new Vector3<FLU>(obj.transform.position);

To convert back, just access the "toUnity" property on the vector.

    Vector3 unityPos = rosPos.toUnity;

And the same functions apply for converting Quaternions.

# Ros Message conversions:

For convenience, `Vector3<C>` has an implicit conversion into all three of the main ROS position message types: Point, Point32 and Vector3. Similarly, `Quaternion<C>` has an implicit conversion to the ROS Quaternion message. Hence, writing 3d data into a message can often be as simple as writing:

	Imu msg = new Imu();
	msg.linear_acceleration = acceleration.To<FLU>();
	msg.orientation = rigidbody.transform.rotation.To<FLU>();
	msg.angular_velocity = rigidbody.angularVelocity.To<FLU>();
	ros.Send("imu", msg);

Note, the calls to `To<FLU>()` above are essential. Normal Unity Vector3s or Quaternions do NOT have these conversions. You need to explicitly select a coordinate frame before converting to a ROS message.

Unity's standard Transform class also has a `To<C>()` extension method that returns a ROS Transform message. So sending a Transform message typically looks like:

    ros.Send("topic", obj.transform.To<FLU>());
