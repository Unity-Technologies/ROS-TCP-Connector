using System;
using UnityEngine;
using UnityEngine.EventSystems;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class RaycastInteraction : Interaction
    {
        protected enum FilterType
        {
            Tag,
            Layer,
            ExactName,
            ContainsName,
            None
        }

        [SerializeField]
        protected string m_RaycastFilter;
        [SerializeField]
        protected FilterType m_FilterType;
        protected Vector3[] m_MouseClicks = new Vector3[2];

        protected bool ValidClick()
        {
            return m_State != ClickState.None && !EventSystem.current.IsPointerOverGameObject();
        }

        /// <summary>
        ///     Returns true if the mouse raycast hit, as well as the RaycastHit
        /// </summary>
        /// <param name="state">ClickState to check raycast during</param>
        /// <returns></returns>
        protected (bool, RaycastHit) RaycastCheck(ClickState state)
        {
            var isHit = Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out var hit) &&
                m_State == state;
            switch (m_FilterType)
            {
                case FilterType.None:
                    return (isHit, hit);
                case FilterType.Tag:
                    isHit = isHit && hit.collider.gameObject.tag.Equals(m_RaycastFilter);
                    break;
                case FilterType.Layer:
                    // TODO
                    // isHit = isHit && hit.collider.gameObject.layer.Equals(m_RaycastFilter);
                    break;
                case FilterType.ExactName:
                    isHit = isHit && hit.collider.gameObject.name.Equals(m_RaycastFilter);
                    break;
                case FilterType.ContainsName:
                    isHit = isHit && hit.collider.gameObject.name.Contains(m_RaycastFilter);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            return (isHit, hit);
        }
    }
}
