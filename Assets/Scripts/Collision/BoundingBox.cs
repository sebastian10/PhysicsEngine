using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoundingBox
{
    public Vector2 boundsMin { private set; get; }
    public Vector2 boundsMax { private set; get; }

    private ICollidable collider;

    public BoundingBox( ICollidable collider )
    {
        this.collider = collider;
    }

    public void Update()
    {
        if ( collider is CircleCollider )
            UpdateBoundsCircle( collider as CircleCollider );
        else if ( collider is RectangleCollider )
            UpdateBoundsRectangle( collider as RectangleCollider );
        else if ( collider is CompoundCollider )
            UpdateBoundsCompound( collider as CompoundCollider );
            
    }

    private void UpdateBoundsCircle( CircleCollider collider )
    {
        Vector2 rotatedOffset = Util.RotateVector2( collider.offset, collider.transform.eulerAngles.z );

        boundsMin = (Vector2) collider.transform.position + rotatedOffset - Vector2.one * Mathf.Abs( collider.radius );
        boundsMax = (Vector2) collider.transform.position + rotatedOffset + Vector2.one * Mathf.Abs( collider.radius );
    }

    private void UpdateBoundsRectangle( RectangleCollider collider )
    {
        Vector2[] vertices = collider.GetVertices();

        boundsMin = new Vector2( Mathf.Infinity, Mathf.Infinity );
        boundsMax = new Vector2( Mathf.NegativeInfinity, Mathf.NegativeInfinity );

        foreach ( Vector2 vertex in vertices )
        {
            SetBoundsMinMaxValues( vertex, vertex );
        }
    }

    private void UpdateBoundsCompound( CompoundCollider collider )
    {
        boundsMin = new Vector2( Mathf.Infinity, Mathf.Infinity );
        boundsMax = new Vector2( Mathf.NegativeInfinity, Mathf.NegativeInfinity );

        collider.colliders.ForEach( col =>
        {
            col.boundingBox.Update();
            SetBoundsMinMaxValues( col.boundingBox.boundsMin, col.boundingBox.boundsMax );
        } );
    }

    private void SetBoundsMinMaxValues( Vector2 boundsMin, Vector2 boundsMax )
    {
        Vector2 boundsMinNew = new Vector2( Mathf.Min( boundsMin.x, this.boundsMin.x ), Mathf.Min( boundsMin.y, this.boundsMin.y ) );
        Vector2 boundsMaxNew = new Vector2( Mathf.Max( boundsMax.x, this.boundsMax.x ), Mathf.Max( boundsMax.y, this.boundsMax.y ) );

        this.boundsMin = boundsMinNew;
        this.boundsMax = boundsMaxNew;
    }
}
