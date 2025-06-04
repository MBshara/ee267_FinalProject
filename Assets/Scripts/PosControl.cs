using UnityEngine;
using UnityEngine.XR;

[RequireComponent(typeof(CharacterController))]
public class PosControl : MonoBehaviour
{
    public Transform vrHead;
    public float speed = 3f;

    CharacterController cc;

    void Awake() => cc = GetComponent<CharacterController>();

    void Update()
    {
        float headHeight = Mathf.Clamp(vrHead.localPosition.y, 1.0f, 2.2f);
        cc.height = headHeight;
        cc.center = new Vector3(vrHead.localPosition.x,
                                headHeight * 0.5f,
                                vrHead.localPosition.z);

        float x = Input.GetAxisRaw("Horizontal"); 
        float z = Input.GetAxisRaw("Vertical");

        // forward = where the headset is looking (flattened to X-Z)
        Vector3 fwd = vrHead.forward;  fwd.y = 0f; fwd.Normalize();
        Vector3 right = vrHead.right;  right.y = 0f; right.Normalize();

        Vector3 move = (fwd * z + right * x).normalized * speed;
        cc.Move(move * Time.deltaTime);
    }
}