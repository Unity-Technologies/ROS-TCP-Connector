using System;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class RaycastPointInteraction : RaycastInteraction
    {
        void Update()
        {
            if (!ValidClick()) return;

            if (Input.GetMouseButtonDown(0))
            {
                var isHit = Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out var hit);
                if (isHit && hit.collider.gameObject.tag.Equals(m_RaycastFilter))
                {
                    Broadcast(hit.point);
                }
            }
        }

        void Broadcast(Vector3 result)
        {
            m_RosPublish.PublishPoint(result);
            m_State = ClickState.None;
            m_RosPublish.ResetTrigger();
        }
    }
}
