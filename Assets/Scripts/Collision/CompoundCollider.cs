using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CompoundCollider : ICollidable
{
    public List<ICollidable> colliders { get; }

    public BoundingBox boundingBox { private set; get; }

    public PhysicsBody attachedPhysicsBody { private set; get; }

    public bool isTrigger { get; }


    public float mass { private set; get; }
    public float momentOfInertia { private set; get; }

    public EventHandler<PhysicsBody> onCollision { get; set; }
    public EventHandler<PhysicsBody> onTrigger { get; set; }


    public CompoundCollider( List<ICollidable> colliders )
    {
        this.colliders = colliders;
        boundingBox = new BoundingBox( this );

        isTrigger = false;

        this.colliders.ForEach( collider =>
        {
            collider.onCollision += ( sender, other ) => { onCollision?.Invoke( this, other ); };
            collider.onTrigger += ( sender, other ) => { onTrigger?.Invoke( this, other ); };
        } );
    }

    public void CalculateStaticParameters()
    {
        mass = 0f;
        colliders.ForEach( collider => { mass += collider.mass; } );

        momentOfInertia = 0f;
        Vector2 centerPoint = GetCenter();

        colliders.ForEach( collider =>
        {
            momentOfInertia += collider.momentOfInertia + collider.mass * (centerPoint - collider.GetCenter()).sqrMagnitude;
        } );
    }

    public void SetAttachedPhysicsBody( PhysicsBody physicsBody ) => attachedPhysicsBody = physicsBody;

    public Vector2 GetCenter()
    {
        Vector2 combinedCenterMass = Vector2.zero;

        colliders.ForEach( collider =>
        {
            combinedCenterMass += collider.GetCenter() * collider.mass;
        } );

        return combinedCenterMass / mass;
    }

    protected void OnDrawGizmos()
    {
        if ( boundingBox == null )
            return;

        boundingBox.Update();

        Gizmos.color = Color.white;
        Gizmos.DrawLine( boundingBox.boundsMin, new Vector2( boundingBox.boundsMin.x, boundingBox.boundsMax.y ) );
        Gizmos.DrawLine( new Vector2( boundingBox.boundsMin.x, boundingBox.boundsMax.y ), boundingBox.boundsMax );
        Gizmos.DrawLine( boundingBox.boundsMax, new Vector2( boundingBox.boundsMax.x, boundingBox.boundsMin.y ) );
        Gizmos.DrawLine( new Vector2( boundingBox.boundsMax.x, boundingBox.boundsMin.y ), boundingBox.boundsMin );
    }
}
