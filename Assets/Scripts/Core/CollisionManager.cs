using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class CollisionManager : MonoBehaviour
{
    List<ICollidable> collidables;

    public static CollisionManager Instance;

    // Collision Data
    private Vector2 contactPoint;
    private Vector2 contactNormal;
    private float penetrationDepth;
    private bool isValidCollision;

    private void Start()
    {
        Instance = this;

        collidables = new List<ICollidable>();
        PhysicsManager.Instance.physicsBodies.ForEach( body =>
        {
            if ( body.collider != null )
                collidables.Add( body.collider );
        } );
    }

    private void FixedUpdate()
    {
        UpdateBoundingBoxes();
        ResolveCollisions();
    }

    private void UpdateBoundingBoxes()
    {
        collidables.ForEach( collidable => { collidable.boundingBox.Update(); } );
    }

    private void ResolveCollisions()
    {
        contactPoint = Vector2.zero;
        contactNormal = Vector2.zero;
        penetrationDepth = 0;
        isValidCollision = false;

        for ( int i = 0; i < collidables.Count; i++ )
        {
            ICollidable collidableA = collidables[i];

            for ( int j = i + 1; j < collidables.Count; j++ )
            {
                ICollidable collidableB = collidables[j];

                ResolveCollision( collidableA, collidableB );
            }
        }
    }

    private void ResolveCollision( ICollidable a, ICollidable b )
    {
        if ( ! ResolveBroadCollision( a.boundingBox, b.boundingBox ) )
            return;

        if ( !ResolveNarrowCollision( a, b ) )
            return;



        if ( a.isTrigger )
            a.onTrigger?.Invoke( this, a.attachedPhysicsBody );

        if ( b.isTrigger )
            b.onTrigger?.Invoke( this, b.attachedPhysicsBody );   


        /*
            Is Valid Collision
            Depenetration and Apply Impulse
        */

        Debug.DrawRay( contactPoint, contactNormal, Color.magenta );

        if ( a.isTrigger || b.isTrigger )
            return;

        a.onCollision?.Invoke( this, a.attachedPhysicsBody );
        b.onCollision?.Invoke( this, b.attachedPhysicsBody );

        CollisionDepenetration( a.attachedPhysicsBody, b.attachedPhysicsBody );
        ApplyImpulse( a.attachedPhysicsBody, b.attachedPhysicsBody );
    }

    private bool ResolveBroadCollision( BoundingBox a, BoundingBox b )
    {
        if ( a.boundsMax.x > b.boundsMin.x && a.boundsMin.x < b.boundsMax.x )
        {
            if ( a.boundsMax.y > b.boundsMin.y && a.boundsMin.y < b.boundsMax.y )
            {
                return true;
            }
        }

        return false;
    }

    private bool ResolveNarrowCollision( ICollidable a, ICollidable b )
    {
        if ( a is CircleCollider && b is CircleCollider )
        {
            return CalculateCollisionData( (CircleCollider) a, (CircleCollider) b );
        }

        if ( a is CircleCollider && b is RectangleCollider )
        {
            return CalculateCollisionData( (CircleCollider) a, (RectangleCollider) b );
        }

        if ( a is RectangleCollider && b is CircleCollider )
        {
            if ( CalculateCollisionData( (CircleCollider) b, (RectangleCollider) a ) )
            {
                contactNormal = -contactNormal;
                return true;
            }

            return false;
        }

        if ( a is RectangleCollider && b is RectangleCollider )
        {
            if ( CalculateCollisionData( (RectangleCollider) b, (RectangleCollider) a ) )
            {
                contactNormal = -contactNormal;
                return true;
            }

            return false;
        }

        if ( a is CompoundCollider )
        {
            CalculateCollisionData( (CompoundCollider) a, b );
            return false;
        }
        else if ( b is CompoundCollider )
        {
            CalculateCollisionData( (CompoundCollider) b, a );
            return false;
        }

        return false;
    }


    /*
        Calculate Collision Data     
    */

    // Circle & Circle
    private bool CalculateCollisionData( CircleCollider a, CircleCollider b )
    {
        Vector2 centerA = a.GetCenter();
        Vector2 centerB = b.GetCenter();
        Vector2 centerDelta = centerB - centerA;
        float combinedRadius = a.radius + b.radius;

        if ( centerDelta.sqrMagnitude <= combinedRadius * combinedRadius )
        {
            contactNormal = centerDelta.normalized;
            penetrationDepth = combinedRadius - centerDelta.magnitude;
            contactPoint = centerA + contactNormal * ( a.radius - penetrationDepth * 0.5f );
            return true;
        }

        return false;
    }

    // Circle & Rectangle
    private bool CalculateCollisionData( CircleCollider a, RectangleCollider b )
    {
        Vector2 centerA = Util.RotateVector2( a.GetCenter() - b.GetCenter(), -b.attachedPhysicsBody.rotation );

        Vector2 inflatedExtents = b.extent + Vector2.one * a.radius;

        if ( centerA.x <= inflatedExtents.x && centerA.x >= -inflatedExtents.x && centerA.y <= inflatedExtents.y && centerA.y >= -inflatedExtents.y )
        {
            Vector2 absCenterA = new Vector2( Mathf.Abs( centerA.x ), Mathf.Abs( centerA.y ) );
            //In Corner
            if ( absCenterA.x >= b.extent.x && absCenterA.y >= b.extent.y )
            {
                if ( ( absCenterA - b.extent ).sqrMagnitude <= a.radius * a.radius )
                {
                    Vector2 mirrorDir = new Vector2( Mathf.Sign( centerA.x ), Mathf.Sign( centerA.y ) );
                    contactPoint = b.extent;
                    contactNormal = ( contactPoint - absCenterA ).normalized * mirrorDir;
                    contactPoint *= mirrorDir;
                    penetrationDepth = a.radius - ( absCenterA - b.extent ).magnitude;
                }
                else
                {
                    return false;
                }
            }
            //Right
            else if ( centerA.x > b.extent.x )
            {
                contactPoint = new Vector2( ( b.extent.x + centerA.x - a.radius ) * 0.5f, centerA.y );
                contactNormal = Vector2.left;
                penetrationDepth = Mathf.Abs( b.extent.x - ( centerA.x - a.radius ) );
            }
            //Left
            else if ( centerA.x < -b.extent.x )
            {
                contactPoint = new Vector2( ( -b.extent.x + centerA.x + a.radius ) * 0.5f, centerA.y );
                contactNormal = Vector2.right;
                penetrationDepth = Mathf.Abs( -b.extent.x - ( centerA.x + a.radius ) );
            }
            //Up
            else if ( centerA.y > b.extent.y )
            {
                contactPoint = new Vector2( centerA.x, ( b.extent.y + centerA.y - a.radius ) * 0.5f );
                contactNormal = Vector2.down;
                penetrationDepth = Mathf.Abs( b.extent.y - ( centerA.y - a.radius ) );
            }
            //Down
            else if ( centerA.y < -b.extent.y )
            {
                contactPoint = new Vector2( centerA.x, ( -b.extent.y + centerA.y + a.radius ) * 0.5f );
                contactNormal = Vector2.up;
                penetrationDepth = Mathf.Abs( -b.extent.y - ( centerA.y + a.radius ) );
            }
            else
            {
                return false;
            }
            contactPoint = b.GetCenter() + Util.RotateVector2( contactPoint, b.attachedPhysicsBody.rotation );
            contactNormal = Util.RotateVector2( contactNormal, b.attachedPhysicsBody.rotation );
            return true;

        }

        return false;
    }

    // Rectangle & Rectangle
    private bool CalculateCollisionData( RectangleCollider a, RectangleCollider b )
    {
        Vector2[] verticesA = new Vector2[4];
        Vector2[] verticesB = new Vector2[4];

        a.GetWorldSpaceVertices( ref verticesA );
        b.GetWorldSpaceVertices( ref verticesB );

        List<CollisionData> collisionDataA = new List<CollisionData>{
            new CollisionData(verticesA[0]),
            new CollisionData(verticesA[1]),
            new CollisionData(verticesA[2]),
            new CollisionData(verticesA[3])
        };

        List<CollisionData> collisionDataB = new List<CollisionData>{
            new CollisionData(verticesB[0]),
            new CollisionData(verticesB[1]),
            new CollisionData(verticesB[2]),
            new CollisionData(verticesB[3])
        };

        //Vertices from B in A
        {
            int p0Index = 3;
            for ( int i = 0; i < 4; i++ )
            {
                Vector2 axis = ( verticesA[i] - verticesA[p0Index] ).normalized;
                Vector2 norm = new Vector2( axis.y, -axis.x );
                for ( int ii = 0; ii < collisionDataB.Count; ii++ )
                {
                    float depth = -Vector2.Dot( norm, collisionDataB[ii].vertexPosition - verticesA[p0Index] );

                    //Discard point if it is outside
                    if ( depth <= 0 )
                    {
                        collisionDataB.RemoveAt( ii );
                        ii--;
                    }
                    else
                    {
                        //Get smallest penetration depth
                        if ( depth < collisionDataB[ii].penetrationDepth )
                        {
                            CollisionData collisionData = collisionDataB[ii];
                            collisionData.penetrationDepth = depth;
                            collisionData.penetrationNormal = norm;
                            collisionDataB[ii] = collisionData;
                        }
                    }

                }
                p0Index = i;
            }
        }

        //Vertices from A in B
        {
            int p0Index = 3;
            for ( int i = 0; i < 4; i++ )
            {
                Vector2 axis = ( verticesB[i] - verticesB[p0Index] ).normalized;
                Vector2 norm = new Vector2( axis.y, -axis.x );
                for ( int ii = 0; ii < collisionDataA.Count; ii++ )
                {
                    float depth = -Vector2.Dot( norm, collisionDataA[ii].vertexPosition - verticesB[p0Index] );

                    //Discard point if it is outside
                    if ( depth <= 0 )
                    {
                        collisionDataA.RemoveAt( ii );
                        ii--;
                    }
                    else
                    {
                        //Get smallest penetration depth
                        if ( depth < collisionDataA[ii].penetrationDepth )
                        {
                            CollisionData collisionData = collisionDataA[ii];
                            collisionData.penetrationDepth = depth;
                            collisionData.penetrationNormal = norm;
                            collisionDataA[ii] = collisionData;
                        }
                    }

                }
                p0Index = i;
            }
        }

        //Get biggest penetration depth
        penetrationDepth = Mathf.NegativeInfinity;

        foreach ( CollisionData collisionData in collisionDataB )
        {
            if ( collisionData.penetrationDepth > penetrationDepth )
            {
                penetrationDepth = collisionData.penetrationDepth;
                contactPoint = collisionData.vertexPosition;
                contactNormal = collisionData.penetrationNormal;
            }
        }

        foreach ( CollisionData collisionData in collisionDataA )
        {
            if ( collisionData.penetrationDepth > penetrationDepth )
            {
                penetrationDepth = collisionData.penetrationDepth;
                contactPoint = collisionData.vertexPosition;
                contactNormal = -collisionData.penetrationNormal;
            }
        }
        
        return penetrationDepth != Mathf.NegativeInfinity;
    }

    // Compound & Other
    private void CalculateCollisionData( CompoundCollider a, ICollidable b )
    {
        a.colliders.ForEach( collider =>
        {
            ResolveCollision( collider, b );
        } );
    }

    private struct CollisionData
    {
        public Vector2 vertexPosition;
        public float penetrationDepth;
        public Vector2 penetrationNormal;

        public CollisionData( Vector2 vertexPosition )
        {
            this.vertexPosition = vertexPosition;
            penetrationDepth = Mathf.Infinity;
            penetrationNormal = Vector2.zero;
        }
    }

    /*
        Resolve Collision
    */

    private void CollisionDepenetration( PhysicsBody a, PhysicsBody b )
    {
        if ( a.isKinematic )
        {
            b.transform.position += (Vector3) ( contactNormal * penetrationDepth );
        }
        else if ( b.isKinematic )
        {
            a.transform.position -= (Vector3) ( contactNormal * penetrationDepth );
        }
        else
        {
            a.transform.position -= (Vector3) ( contactNormal * penetrationDepth * b.mass / ( a.mass + b.mass ) );
            b.transform.position += (Vector3) ( contactNormal * penetrationDepth * a.mass / ( a.mass + b.mass ) );
        }

        a.collider.boundingBox.Update();
        b.collider.boundingBox.Update();
    }

    private void ApplyImpulse( PhysicsBody a, PhysicsBody b )
    {
        Vector2 contactVelocityA = a.GetVelocityOfPoint( contactPoint );
        Vector2 contactVelocityB = b.GetVelocityOfPoint( contactPoint );

        Vector2 tangent = new Vector2( -contactNormal.y, contactNormal.x );

        Vector2 contactDeltaVelocity = contactVelocityB - contactVelocityA;
        float projectedContactDeltaVelocity = Vector2.Dot( contactDeltaVelocity, contactNormal );

        if ( projectedContactDeltaVelocity < 0 )
        {
            //Direct
            float ndA = Vector2.Dot( tangent, contactPoint - a.GetCenter() );
            float ndB = Vector2.Dot( tangent, contactPoint - b.GetCenter() );

            float p = projectedContactDeltaVelocity / ( ( 1 / a.mass ) + ( 1 / b.mass ) + ( ndA * ndA / a.momentOfInertia ) + ( ndB * ndB / b.momentOfInertia ) );

            float combinedRestitution = a.restitution * b.restitution;

            p *= ( 1 + combinedRestitution );

            a.velocity += contactNormal * p / a.mass;
            b.velocity -= contactNormal * p / b.mass;

            a.angularVelocity -= p * ndA / a.momentOfInertia;
            b.angularVelocity += p * ndB / b.momentOfInertia;


            //Friction
            float projectedContactDeltaVelocityFriction = Vector2.Dot( contactDeltaVelocity, tangent );
            float ndAFriction = Vector2.Dot( contactNormal, contactPoint - a.GetCenter() );
            float ndBFriction = Vector2.Dot( contactNormal, contactPoint - b.GetCenter() );

            float pFriction = projectedContactDeltaVelocityFriction / ( ( 1 / a.mass ) + ( 1 / b.mass ) + ( ndAFriction * ndAFriction / a.momentOfInertia ) + ( ndBFriction * ndBFriction / b.momentOfInertia ) );

            float combinedFriction = a.friction * b.friction;
            float pMaxFriction = Mathf.Abs( p * combinedFriction );

            pFriction = Mathf.Clamp( pFriction, -pMaxFriction, pMaxFriction );

            a.velocity += tangent * pFriction / a.mass;
            b.velocity -= tangent * pFriction / b.mass;

            a.angularVelocity += pFriction * ndAFriction / a.momentOfInertia;
            b.angularVelocity -= pFriction * ndBFriction / b.momentOfInertia;
        }
    }


    protected virtual void OnDrawGizmos()
    {
        collidables.ForEach( collidable =>
        {
            if ( collidable.boundingBox == null )
                return; 

            if ( collidable is CompoundCollider )
            {
                CompoundCollider cc = collidable as CompoundCollider;
                cc.colliders.ForEach( col => { DrawBoundingBox( col.boundingBox ); } );
            }

            DrawBoundingBox( collidable.boundingBox );

        } );   
    }

    private void DrawBoundingBox( BoundingBox boundingBox )
    {
        Gizmos.color = Color.white;
        Gizmos.DrawLine( boundingBox.boundsMin, new Vector2( boundingBox.boundsMin.x, boundingBox.boundsMax.y ) );
        Gizmos.DrawLine( new Vector2( boundingBox.boundsMin.x, boundingBox.boundsMax.y ), boundingBox.boundsMax );
        Gizmos.DrawLine( boundingBox.boundsMax, new Vector2( boundingBox.boundsMax.x, boundingBox.boundsMin.y ) );
        Gizmos.DrawLine( new Vector2( boundingBox.boundsMax.x, boundingBox.boundsMin.y ), boundingBox.boundsMin );
    }
}
