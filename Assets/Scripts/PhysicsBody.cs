using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class PhysicsBody : MonoBehaviour
{
    // Settings
    [Header( "Settings" )]
    public float density = 1f;
    public float linearDrag = 0.5f;
    public float angularDrag = 0.1f;
    public float gravityScale = 1f;

    public bool isKinematic = false;

    [Space(20)]
    [Range(0, 1)]
    public float restitution; // Bounciness
    public float friction;

    // Runtime Values
    [Space(20)]
    public Vector2 velocity;
    public float angularVelocity;

    public float mass { set; get; } = 1f;
    public float momentOfInertia { set; get; }


    // Collider
    private List<ICollidable> colliders = new List<ICollidable>();
    public new ICollidable collider { private set; get; }
        

    public float rotation
    {
        get { return transform.eulerAngles.z; }
    }

    public Vector2 GetCenter() => collider.GetCenter();

    public Vector2 GetVelocityOfPoint( Vector2 point )
    {
        Vector2 delta = point - GetCenter();

        return velocity + new Vector2( -delta.y, delta.x ) * angularVelocity;
    }


    private void Start()
    {
        SetCollider();
    }

    private void SetCollider()
    {
        colliders = GetComponents<ICollidable>().ToList();

        if ( colliders.Count <= 0 )
            return;

        if ( colliders.Count == 1 )
            collider = colliders[0];
        else
            collider = new CompoundCollider( colliders );


        // Calculate Static Parameters for every collider
        colliders.ForEach( collider => SetUpCollider( collider ) );

        if ( collider is CompoundCollider )
            SetUpCollider( collider );

        // Set Static Parameters for the physics body
        mass = isKinematic ? Mathf.Infinity : collider.mass;
        momentOfInertia = isKinematic ? Mathf.Infinity : collider.momentOfInertia;


        // OnCollision Events Setup
        collider.onCollision += ICollidable_OnCollision;
        collider.onTrigger += ICollidable_OnTrigger;
    }

    private void SetUpCollider( ICollidable collider )
    {
        collider.SetAttachedPhysicsBody( this );
        collider.CalculateStaticParameters();
    }


    public void ICollidable_OnCollision( object sender, PhysicsBody other )
    {
        Debug.Log( $"Collision between - { this.gameObject } & { other.gameObject }" );
    }

    public void ICollidable_OnTrigger( object sender, PhysicsBody other )
    {
        Debug.Log( $"Trigger collision between - { this.gameObject } & { other.gameObject }" );
    }


    /*
        Forces
    */
    public void AddForce( Vector2 force )
    {
        velocity += force * Time.fixedDeltaTime / mass;
    }

    public void AddImpulse( Vector2 impulse )
    {
        velocity += impulse / mass;
    }

    public void AddTorque( float torque )
    {
        angularVelocity += torque * Time.fixedDeltaTime / momentOfInertia;
    }

    public void AddAngularImpulse( float angularImpulse )
    {
        angularVelocity += angularImpulse / momentOfInertia;
    }


    public void AddForceAtPosition( Vector2 force, Vector2 position )
    {
        velocity += force * Time.fixedDeltaTime / mass;

        Vector2 delta = position - GetCenter();

        float dist = Vector2.Dot( new Vector2( -force.y, force.x ).normalized, delta );

        float M = -force.magnitude * dist;

        angularVelocity += M * Time.fixedDeltaTime / momentOfInertia;
    }

    public void AddImpulseAtPosition( Vector2 impulse, Vector2 position )
    {
        velocity += impulse / mass;

        Vector2 delta = position - GetCenter();

        float dist = Vector2.Dot( new Vector2( -impulse.y, impulse.x ).normalized, delta );

        float M = -impulse.magnitude * dist;

        angularVelocity += M / momentOfInertia;
    }



    private void OnEnable()
    {
        PhysicsManager.Instance.AddBody( this );
    }

    private void OnDisable()
    {
        if ( ! PhysicsManager.hasValidInstance )
            return;

        PhysicsManager.Instance.RemoveBody( this );
    }

    private void OnDrawGizmos()
    {
        if ( collider == null )
            return;

        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere( collider.GetCenter(), 0.1f );
    }
}
