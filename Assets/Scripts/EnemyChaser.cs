using UnityEngine;

[RequireComponent(typeof(Collider))]
public class EnemyChaser : MonoBehaviour
{
    public Transform target;

    [Header("Movement")]
    public float speed = 2.0f;
    public bool lockY = true;

    [Header("Bobbing")]
    public float bobAmplitude = 0.25f;
    public float bobFrequency = 1.0f;

    float baseY; // starting vertical position

    void Start(){
        baseY = transform.position.y;

        Collider col = GetComponent<Collider>();
        if (col && !col.isTrigger) col.isTrigger = true;
    }

    void Update()
    {
        if (!target) return;

        Vector3 toTarget = target.position - transform.position;
        if (lockY) toTarget.y = 0f; // ignore vertical difference if desired
        Vector3 horizDir = toTarget.normalized;
        transform.position += horizDir * speed * Time.deltaTime;

        float offsetY = Mathf.Sin(Time.time * bobFrequency * Mathf.PI * 2f) * bobAmplitude;
        Vector3 pos = transform.position;
        pos.y = (lockY ? baseY : target.position.y) + offsetY;
        transform.position = pos;

        if (horizDir.sqrMagnitude > 0.0001f) // facing target
            transform.rotation = Quaternion.LookRotation(horizDir);
    }
}
