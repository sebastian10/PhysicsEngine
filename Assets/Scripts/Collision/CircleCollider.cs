using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CircleCollider : BaseCollider
{
    // Settings
    public float radius = 1f;

    public override void CalculateStaticParameters()
    {
        mass = radius * radius * Mathf.PI * attachedPhysicsBody.density;
        momentOfInertia = mass * radius * radius * .5f;
    }

    protected void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere( transform.position + (Vector3) Util.RotateVector2( offset, transform.eulerAngles.z ), radius );
    }
}
