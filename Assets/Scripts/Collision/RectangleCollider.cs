using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RectangleCollider : BaseCollider
{
    public Vector2 extent;

    public override void CalculateStaticParameters()
    {
        mass = extent.x * extent.y * 4 * attachedPhysicsBody.density;
        momentOfInertia = mass * ( extent.x * extent.x * 4 + extent.y * extent.y * 4 ) / 12;
    }

    public Vector2[] GetVertices()
    {
        float angle = transform.eulerAngles.z;
        Vector2 pointCenter = (Vector2) transform.position + Util.RotateVector2( offset, angle );

        return new Vector2[] {
            pointCenter + Util.RotateVector2( new Vector2(  extent.x,  extent.y ), angle ),
            pointCenter + Util.RotateVector2( new Vector2(  extent.x, -extent.y ), angle ),
            pointCenter + Util.RotateVector2( new Vector2( -extent.x, -extent.y ), angle ),
            pointCenter + Util.RotateVector2( new Vector2( -extent.x,  extent.y ), angle )
        };
    }

    public void GetWorldSpaceVertices( ref Vector2[] vertices )
    {
        Vector2 center = GetCenter();
        Vector2 corner0 = Util.RotateVector2( extent, attachedPhysicsBody.rotation );
        Vector2 corner1 = Util.RotateVector2( new Vector2( -extent.x, extent.y ), attachedPhysicsBody.rotation );

        vertices[0] = center + corner0;
        vertices[1] = center + corner1;
        vertices[2] = center - corner0;
        vertices[3] = center - corner1;
    }

    private void OnDrawGizmos()
    {
        Vector2[] vertices = GetVertices();

        Gizmos.color = Color.green;

        int p1 = vertices.Length - 1;
        for ( int p2 = 0; p2 < vertices.Length; p2++ )
        {
            Gizmos.DrawLine( vertices[p2], vertices[p1] );
            p1 = p2;
        }
    }
}
