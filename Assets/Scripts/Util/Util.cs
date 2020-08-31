using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Util
{
    public static Vector2 RotateVector2( Vector2 vector, float degree )
    {
        float s = Mathf.Sin( degree * Mathf.Deg2Rad );
        float c = Mathf.Cos( degree * Mathf.Deg2Rad );

        return new Vector2( vector.x * c - vector.y * s, vector.y * c + vector.x * s );
    }
}
