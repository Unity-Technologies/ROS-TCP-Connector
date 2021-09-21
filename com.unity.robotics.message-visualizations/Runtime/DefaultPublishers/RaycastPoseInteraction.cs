using System;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class RaycastPoseInteraction : RaycastInteraction
    {
        [SerializeField]
        Color m_Color;
        Drawing3d m_ArrowDrawing;

        public override void Start()
        {
            base.Start();
            m_ArrowDrawing = Drawing3dManager.CreateDrawing();
        }

        void Update()
        {
            if (!ValidClick()) return;

            if (Input.GetMouseButtonDown(0)) // Begin click
            {
                BeginClick();
            }

            if (Input.GetMouseButton(0)) // Update arrow during drag
            {
                UpdateClick();
            }

            if (Input.GetMouseButtonUp(0)) // Release click
            {
                ReleaseClick();
            }
        }

        void BeginClick()
        {
            var (didHit, hit) = RaycastCheck(ClickState.Started);
            if (didHit)
            {
                m_MouseClicks[0] = hit.point;
                m_State = ClickState.Held;
            }
        }

        void UpdateClick()
        {
            var (didHit, hit) = RaycastCheck(ClickState.Held);
            if (didHit)
            {
                m_ArrowDrawing.Clear();

                // Draw normalized arrow in direction of mouse position
                m_ArrowDrawing.DrawArrow(m_MouseClicks[0],
                    1.5f * Vector3.Normalize(hit.point - m_MouseClicks[0]) + m_MouseClicks[0], m_Color,
                    arrowheadScale: 2f);
            }
        }

        void ReleaseClick()
        {
            m_ArrowDrawing.Clear();
            var (didHit, hit) = RaycastCheck(ClickState.Held);
            if (didHit)
            {
                m_MouseClicks[1] = hit.point;
                m_State = ClickState.None;
                Broadcast(CalculatePose());
            }
        }

        /// <summary>
        ///     Calculates and formats pose based on mouse interactions
        /// </summary>
        (Vector3, Quaternion) CalculatePose()
        {
            var diff = (m_MouseClicks[1] - m_MouseClicks[0]).normalized;
            var rot = diff == Vector3.zero ? Quaternion.identity : Quaternion.LookRotation(diff);
            return (m_MouseClicks[0], rot);
        }

        void Broadcast((Vector3, Quaternion) result)
        {
            m_RosPublish.PublishPose(result);
            m_State = ClickState.None;
            m_RosPublish.ResetTrigger();
        }
    }
}
