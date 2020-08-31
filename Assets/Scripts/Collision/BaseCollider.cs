using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class BaseCollider : MonoBehaviour, ICollidable
{
    [Header("Settings")]
    public Vector2 offset;

    public BoundingBox boundingBox { protected set; get; }

    public PhysicsBody attachedPhysicsBody { protected set; get; }

    [SerializeField]
    private bool IsTrigger;
    public bool isTrigger 
    {
        get
        {
            return IsTrigger;
        } 
    }


    public float mass { protected set; get; }
    public float momentOfInertia { protected set; get; }

    public EventHandler<PhysicsBody> onCollision { get; set; }
    public EventHandler<PhysicsBody> onTrigger { get; set; }


    public void SetAttachedPhysicsBody( PhysicsBody body ) => attachedPhysicsBody = body;
    public abstract void CalculateStaticParameters();

    public virtual Vector2 GetCenter()
    {
        return (Vector2) transform.position + Util.RotateVector2( offset, transform.eulerAngles.z );
    }

    private void Awake()
    {
        boundingBox = new BoundingBox( this );
    }
}
