using System;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{

    public static class CameraInfoGenerator
    {

        //The default Camera Info distortion model.
        const string k_PlumbBobDistortionModel = "plumb_bob";

        /**
         * <summary>
         *   <para>Using a Unity Camera and a provided HeaderMsg, generate a corresponding CameraInfoMsg,
         *  see http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html for more information.</para>
         * </summary>
         *
         * <param name="unityCamera">The camera used to generate the message.</param>
         * <param name="header">The header component of this message.</param>
         * <param name="horizontalCameraOffsetDistanceMeters">Meters, For use with stereo cameras,
         * Typically the offset of the right camera from the left.</param>
         * <param name="integerResolutionTolerance">The CameraInfoMsg stores resolution as a uint. However in Unity the
         * pixelRect that represents the resolution is a Rect allowing for floating point precision resolution.
         * This parameter will cause debug warnings if after converting a resolution to unsigned integers
         * the difference between the new value and the original is greater than integerResolutionTolerance.
         * If you want to bypass this warning, set the value to 1.0f</param>
         *
         * Not Yet Supported, but might be worth supporting:
         * -> Camera Resolution Scaling.
         * -> Non Zero Vertical Camera Offset.
         * -> Lens Shift.
         *
         * Not Yet Supported:
         * -> Distortion Models.
         * -> Rectification Matrix (Other than the identity).
         * -> Axis Skew.
         *
         */
        public static CameraInfoMsg ConstructCameraInfoMessage(Camera unityCamera, HeaderMsg header,
            float horizontalCameraOffsetDistanceMeters = 0.0f, float integerResolutionTolerance = 0.01f)
        {
            var cameraInfo = new CameraInfoMsg { header = header };

            Rect pixelRect = unityCamera.pixelRect;
            var resolutionWidth = (uint)pixelRect.width;
            var resolutionHeight = (uint)pixelRect.height;

            //Check whether the resolution is an integer value, if not, raise a warning.
            //Note: While the resolution of a screen or a render texture is always an integer value,
            //      one can change the rendering region within the screen / texture using the
            //      viewport rect. It is possible that this region will be a non-integer resolution.
            //      since the resolution of the CameraInfo message is stored as a uint,
            //      non-integer values are not supported
            if ((pixelRect.width - (float)resolutionWidth) > integerResolutionTolerance)
            {
                Debug.LogWarning($"CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                                 $"Resolution width is not an integer: {pixelRect.width}. Adjust the viewport rect.");
            }

            if ((pixelRect.height - (float)resolutionHeight) > integerResolutionTolerance)
            {
                Debug.LogWarning($"CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                                 $"Resolution height is not an integer: {pixelRect.height}. Adjust the viewport rect.");
            }

            if (resolutionWidth != unityCamera.scaledPixelWidth || resolutionHeight != unityCamera.scaledPixelHeight)
            {
                //Check for resolution scaling (Not Implemented). TODO - Implement.
                throw new NotImplementedException(
                    $"Unable to construct CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                    $"Resolution scaling is not yet supported.");
            }

            if (unityCamera.lensShift != Vector2.zero)
            {
                throw new NotImplementedException(
                    $"Unable to construct CameraInfoMsg for camera with name {unityCamera.gameObject.name}, " +
                    "Lens shift is not yet supported.");
            }

            cameraInfo.width = resolutionWidth;
            cameraInfo.height = resolutionHeight;

            //Focal center currently assumes zero lens shift.
            //Focal center x.
            double cX = resolutionWidth / 2.0;
            //Focal center y.
            double cY = resolutionHeight / 2.0;

            //Get the vertical field of view of the camera taking into account any physical camera settings.
            float verticalFieldOfView = GetVerticalFieldOfView(unityCamera);

            //Sources
            //http://paulbourke.net/miscellaneous/lens/
            //http://ksimek.github.io/2013/06/18/calibrated-cameras-and-gluperspective/
            //Rearranging the equation for verticalFieldOfView given a focal length, determine the focal length in pixels.
            double focalLengthInPixels =
                (resolutionHeight / 2.0) / Math.Tan((Mathf.Deg2Rad * verticalFieldOfView) / 2.0);

            //As this is a perfect pinhole camera, the fx = fy = f
            //Source http://ksimek.github.io/2013/08/13/intrinsic/
            //Focal Length (x)
            double fX = focalLengthInPixels;
            //Focal Length (y)
            double fY = focalLengthInPixels;

            //Source: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
            //For a single camera, tX = tY = 0.
            //For a stereo camera, assuming Tz = 0, Ty = 0 and Tx = -fx' * B (for the second camera)
            double baseline = horizontalCameraOffsetDistanceMeters;

            double tX = -fX * baseline;
            double tY = 0.0;

            //Axis Skew, Assuming none.
            double s = 0.0;

            //http://ksimek.github.io/2013/08/13/intrinsic/
            cameraInfo.k = new double[]
            {
                fX, s, cX,
                0, fY, cY,
                0, 0, 1
            };

            //The distortion parameters, size depending on the distortion model.
            //For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
            //No distortion means d = {k1, k2, t1, t2, k3} = {0, 0, 0, 0, 0}
            cameraInfo.distortion_model = k_PlumbBobDistortionModel;
            cameraInfo.d = new double[]
            {
                0.0, //k1
                0.0, //k2
                0.0, //t1
                0.0, //t2
                0.0 //k3
            };

            //Rectification matrix (stereo cameras only)
            //A rotation matrix aligning the camera coordinate system to the ideal
            //stereo image plane so that epipolar lines in both stereo images are
            //parallel.
            cameraInfo.r = new double[]
            {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
            };


            //Projection/camera matrix
            //     [fx'  0  cx' Tx]
            // P = [ 0  fy' cy' Ty]
            //     [ 0   0   1   0]
            cameraInfo.p = new double[]
            {
                fX, 0, cX, tX,
                0, fY, cY, tY,
                0, 0, 1, 0
            };

            //We're not worrying about binning...
            cameraInfo.binning_x = 0;
            cameraInfo.binning_y = 0;

            cameraInfo.roi = new RegionOfInterestMsg(); //Just the default values here.

            return cameraInfo;
        }

        /**
         * The Unity Camera has a useful property 'fieldOfView' that should be
         * "The vertical field of view of the Camera, in degrees.". However, when
         * 'usePhysicalProperties' is enabled, the vertical field of view is the
         * field of view of the sensor but not the rendered image.
         * This is influenced by the 'gateFit' and this function aims to take this
         * into account and return the vertical field of view of the rendered image
         * no matter the gateFit.
         * <summary>
         *   <para>The vertical field of view of the rendered image taking into account physical properties, in degrees.</para>
         * </summary>
         */
        private static float GetVerticalFieldOfView(Camera camera)
        {
            if (camera.usePhysicalProperties)
            {
                //The gateFit may influence the vertical field of view.
                Vector2 sensorSize = camera.sensorSize;
                Rect pixelRect = camera.pixelRect;

                float sensorRatioY = sensorSize.y / sensorSize.x;
                float pixelRatioY = pixelRect.height / pixelRect.width;
                float fovMultiplier = pixelRatioY / sensorRatioY;

                switch (camera.gateFit)
                {
                    case Camera.GateFitMode.Vertical:
                        //The fieldOfView from the camera is accurate, return it.
                        return camera.fieldOfView;
                    case Camera.GateFitMode.Horizontal:
                        //The fieldOfView from the camera is influenced by the ratio of the pixels vs the sensor size ratio.
                        return camera.fieldOfView * fovMultiplier;
                    case Camera.GateFitMode.Fill:
                        if (fovMultiplier >= 1.0f)
                        {
                            //Same as GateFitMode.Vertical
                            return camera.fieldOfView;
                        }
                        else
                        {
                            //Same as GateFitMode.Horizontal
                            return camera.fieldOfView * fovMultiplier;
                        }
                    case Camera.GateFitMode.Overscan:
                        if (fovMultiplier <= 1.0f)
                        {
                            //Same as GateFitMode.Vertical
                            return camera.fieldOfView;
                        }
                        else
                        {
                            //Same as GateFitMode.Horizontal
                            return camera.fieldOfView * fovMultiplier;
                        }
                    case Camera.GateFitMode.None:
                        //The view is stretched, the fieldOfView is valid.
                        return camera.fieldOfView;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }

            //The fieldOfView from the camera is accurate, return it.
            return camera.fieldOfView;
        }
    }
}
