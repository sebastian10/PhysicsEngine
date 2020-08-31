using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsManager : MonoBehaviour
{
    public List<PhysicsBody> physicsBodies { get; } = new List<PhysicsBody>();

    public CollisionManager collisionManager;

    public readonly Vector2 GRAVITY = new Vector2( 0, -1.81f );

    // Physics Manager Instance stuff
    private static PhysicsManager _instance;
    public static PhysicsManager Instance
    {
        get
        {
            if ( !_instance )
            {
                GameObject go = new GameObject();
                go.name = "_PhysicsManager";
                _instance = go.AddComponent<PhysicsManager>();
            }

            return _instance;
        }
    }

    public static bool hasValidInstance { 
        get { return _instance != null; } 
    }

    public void AddBody( PhysicsBody physicsBody ) => physicsBodies.Add( physicsBody );
    public void RemoveBody( PhysicsBody physicsBody ) => physicsBodies.Remove( physicsBody );


    private void Start()
    {
        if ( collisionManager != null )
            Destroy( collisionManager.gameObject );

        GameObject go = new GameObject();
        go.name = "_CollisionManager";
        go.transform.parent = transform;
        collisionManager = go.AddComponent<CollisionManager>();
    }

    private void FixedUpdate()
    {
        SolveVelocities();
    }

    private void SolveVelocities()
    {
        foreach ( PhysicsBody physicsBody in physicsBodies )
        {
            if ( physicsBody.isKinematic )
                continue;

            if ( physicsBody.velocity.x <= 0.05f && physicsBody.velocity.x >= -0.05f )
                physicsBody.velocity.x = 0;

            if ( physicsBody.velocity.y <= 0.05f && physicsBody.velocity.y >= -0.05f )
                physicsBody.velocity.y = 0;

            Transform t = physicsBody.transform;
            physicsBody.velocity += GRAVITY * physicsBody.gravityScale;

            // Drag
            physicsBody.velocity *= GetDrag( physicsBody.linearDrag );
            physicsBody.angularVelocity *= GetDrag( physicsBody.angularDrag );

            t.position += (Vector3) physicsBody.velocity * Time.fixedDeltaTime;
            t.RotateAround( physicsBody.GetCenter(), Vector3.forward, physicsBody.angularVelocity * Mathf.Rad2Deg * Time.fixedDeltaTime );
        }
    }

    private float GetDrag( float drag )
    {
        float multiplier = 1f - drag * Time.fixedDeltaTime;
        return multiplier < 0 ? 0 : multiplier;
    }
}
