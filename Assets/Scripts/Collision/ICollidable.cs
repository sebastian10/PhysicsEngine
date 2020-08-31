using UnityEngine;
using System;

public interface ICollidable
{
    BoundingBox boundingBox { get; }
    PhysicsBody attachedPhysicsBody { get; }

    bool isTrigger { get; }

    float mass { get; }
    float momentOfInertia { get; }

    EventHandler<PhysicsBody> onCollision { set; get; }
    EventHandler<PhysicsBody> onTrigger { set; get; }

    void SetAttachedPhysicsBody( PhysicsBody physicsBody );
    void CalculateStaticParameters();
    Vector2 GetCenter();
}
