# ROSGeometry component

In Unity, the X axis points right, Y up, and Z forward. ROS, on the other hand, supports various coordinate frames: in the most commonly-used one, X points forward, Y left, and Z up. In ROS terminology, this frame is called "FLU" (forward, left, up), whereas the Unity coordinate frame would be "RUF" (right, up, forward).

The ROSGeometry namespace contains code to make it easier to work with these various coordinate frames - letting you be explicit about what coordinates a given value is in at compile time, and managing the conversions for you.

Here's the using declaration for adding ROSGeometry to your namespace:

    using Unity.Robotics.ROSTCPConnector.ROSGeometry;

# Ros Message conversions:

The main ROS position messages (geometry_msgs/Point, geometry_msgs/Point32 and geometry_msgs/Vector3) can be converted to and from Unity Vector3s like this:

	PointMsg ToRosExample(Vector3 position)
	{
		return position.To<FLU>();
	}

	Vector3 ToUnityExample(PointMsg message)
	{
		return message.From<FLU>();
	}

Similarly, the geometry_msgs/Quaternion message can be converted to and from a Unity Quaternion.

	QuaternionMsg myRosQuaternion = myGameObject.transform.rotation.To<FLU>();
	Quaternion myUnityQuaternion = myRosQuaternion.From<FLU>();

Hence, writing 3d data into a message can often be as simple as writing:

	ImuMsg msg = new ImuMsg()
	{
		linear_acceleration = acceleration.To<FLU>();
		orientation = rigidbody.transform.rotation.To<FLU>();
		angular_velocity = rigidbody.angularVelocity.To<FLU>();
	}
	ros.Send("imu", msg);

Unity's standard Transform class also has a `To<C>()` extension method that returns a ROS Transform message. So creating a geometry_msgs/Transform message typically looks like this:

    TransformMsg msg = myGameObject.transform.To<FLU>());

# Internal details

Some more detail about what's going on here: The core of the ROSGeometry package is the two generic structs, `Vector3<C>` and `Quaternion<C>`. The type parameter C here indicates the coordinate frame you're working in - either FLU, or RUF, or perhaps one of the more exotic frames such as NED (north, east, down) or ENU (east, north, up), used in aviation.

These are fully-fledged Vector3 and Quaternion classes, so if you want, you can work with them directly to perform geometric operations in an arbitrary coordinate space. (Note, it's a compile time error to add a Vector3<FLU> to a Vector3<RUF>.)

These are the types returned by the `To<FLU>()` calls above. Vector3<C> also has implicit conversions to MPoint, MPoint32, and MVector3, which is how this one call can be used to convert to all three data types.

# Converting between frames:

If, for example, you need to convert an object's position into the FLU coordinate frame, you might explicitly create a Vector3<FLU> like this:

    Vector3<FLU> rosPos = obj.transform.position.To<FLU>();

An explicit cast, or calling the constructor, would also produce the same result.

    Vector3<FLU> rosPos2 = (Vector3<FLU>)obj.transform.position;
    Vector3<FLU> rosPos3 = new Vector3<FLU>(obj.transform.position);

To convert back, just access the "toUnity" property on the vector.

    Vector3 unityPos = rosPos.toUnity;

The same functions and properties apply for converting Quaternions.

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

# Converting incoming messages

You can also convert Points, Point32s and Vector3s back into Unity coordinates. If you know that a Point is in coordinate space C, you can convert it into a Unity Vector3 in the unity coordinate space by writing `From<C>`. For example:

    void SubscriberCallback(Point p)
	{
	  transform.position = p.From<FLU>();
	}

Or, if you need to work with it in the FLU coordinate space for now, you can write:

    Vector3<FLU> rosPos = p.As<FLU>();

(Note, the As function does NOT do any coordinate conversion. It simply assumes the point is in the FLU coordinate frame already, and transfers it into an appropriate container.)

And again, the same goes for converting a Quaternion message into a Unity Quaternion or `Quaternion<C>`.

# Geographical Coordinates

The geographical coordinate systems NED and ENU are more complicated than the rest: besides the obvious "north" direction, they also provide a convention for which direction is "forward", which is not even consistent between them - In NED, forward (i.e. yaw 0) is north, whereas in ENU forward is east. Because of this inconsistency, there's no universal convention that we could establish for which direction is north in Unity. Instead, we're forced to make a distinction between local and world rotations.

To correctly convert a *world* position or rotation to or from NED or ENU coordinates, you should use the standard NED or ENU coordinate space - for example, `To<NED>()`. This conversion will respect what direction you have set as North - by default, the Unity Z axis points north, but is a configuration option that you can change in the Robotics/ROS Settings menu.

If you have a *local* rotation represented in NED or ENU coordinates, you should convert it using the coordinate space NEDLocal or ENULocal: for example, `To<NEDLocal>()`. These conversions are not appropriate for handling world rotations, because they don't respect the north direction, only what direction is forward (i.e. the Unity Z axis). In other words, with NEDLocal and ENULocal, an identity quaternion is still identity. This is not necessarily true for NED or ENU. NEDLocal is a synonym for the FRD coordinate space (north = forward, east = right, down = down) and ENULocal is a synonym for the FLU coordinate space (east = forward, north = left, up = up).

If you only care about NED coordinates, you can avoid having to deal with this distinction by setting the Z axis direction to North (the default) - this will make the NED coordinate space equal to NEDLocal. Similarly, if you only care about ENU coordinates, setting the Z axis direction to East will make ENU equal to ENULocal.
