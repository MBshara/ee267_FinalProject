using UnityEngine;
using UnityEngine.SceneManagement;
using TMPro;

public class GhostGameOverTrigger : MonoBehaviour
{
    [Header("References")]
    public Transform ghost;
    public Transform playerCamera; 
    public TextMeshPro gameOverText3D; 
    public ReadUSB usbRotationScript;
    public MonoBehaviour[] scriptsToDisable;

    [Header("Tuning")]
    public float triggerDistance = 2f;
    public float rotateSpeed =1f;
    
    [Header("3D Text Settings")]
    public float textDistanceFromPlayer = 3f;
    public float textHeight = 1.8f;
    public bool textFollowsPlayer = true;
    public float textScale = 0.1f;

    bool isGameOver = false;
    float gameOverStartTime;
    Vector3 fixedTextPosition;

    void Start()
    {
        if (gameOverText3D != null) // Hide text before game starts
            gameOverText3D.gameObject.SetActive(false);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Y)){
            RestartGame();
            return;
        }
        
        if (isGameOver){
            RotateCameraTowardsGhost();
            UpdateTextPosition();
            MakeTextFaceCamera();
            return;
        }

        // raw 3D diff
        Vector3 diff3D = ghost.position - transform.position;

        // ignore Y
        Vector3 diffXZ = new Vector3(diff3D.x, 0f, diff3D.z);
        float distXZ   = diffXZ.magnitude;

        //  debug only XZ
        if (distXZ <= triggerDistance)
        {
            Debug.LogWarning(
                $"[GhostGameOver] XZ diff: ({diffXZ.x:F2}, {diffXZ.z:F2})  distXZ = {distXZ:F2}"
            );
            TriggerGameOver();
        }
    }

    void TriggerGameOver()
    {
        isGameOver = true;
        gameOverStartTime = Time.time;

        Debug.Log("[GhostGameOver] Game Over triggered! Disabling movement...");

        // I dont think this works...
        if (usbRotationScript != null)
        {
            usbRotationScript.enabled = false;
            Debug.Log("[GhostGameOver] Disabled USB rotation script");
        }

        // THIS WORKS: Stop any other movement/input scripts
        if (scriptsToDisable != null){
            foreach (var s in scriptsToDisable){
                if (s != null) {
                    s.enabled = false;
                    Debug.Log($"[GhostGameOver] Disabled script: {s.GetType().Name}");
                }
            }
        }
        else{
            Debug.LogWarning("[GhostGameOver] No scripts assigned to scriptsToDisable array!");
        }

        // Turn off ability to look around
        if (TryGetComponent<CharacterController>(out var cc)){
            cc.enabled = false;
            Debug.Log("[GhostGameOver] Disabled CharacterController");
        }

        Show3DGameOverText();
    }

    void Show3DGameOverText()
    {
        if (gameOverText3D != null){
            gameOverText3D.text = "You have been caught!";
            gameOverText3D.gameObject.SetActive(true);
            
            if (!textFollowsPlayer){
                Vector3 forward = playerCamera.forward;
                forward.y = 0;
                forward.Normalize();
                
                fixedTextPosition = transform.position + forward * textDistanceFromPlayer;
                fixedTextPosition.y = textHeight;
                gameOverText3D.transform.position = fixedTextPosition;
            }
            
            gameOverText3D.transform.localScale = Vector3.one * textScale;
            
            gameOverText3D.fontSize = 36;
            gameOverText3D.alignment = TextAlignmentOptions.Center;
            gameOverText3D.color = Color.red;
            gameOverText3D.fontMaterial.SetFloat("_OutlineWidth", 0.2f);
            gameOverText3D.fontMaterial.SetColor("_OutlineColor", Color.black);
        }
    }

    void UpdateTextPosition(){
        if (gameOverText3D == null || !gameOverText3D.gameObject.activeInHierarchy) 
            return;

        if (textFollowsPlayer)
        {
            Vector3 forward = playerCamera.forward;
            forward.y = 0;
            forward.Normalize();
            
            Vector3 textPos = playerCamera.position + forward * textDistanceFromPlayer;
            textPos.y = textHeight;
            gameOverText3D.transform.position = textPos;
        }
        else{
            gameOverText3D.transform.position = fixedTextPosition;
        }
    }

    void MakeTextFaceCamera()
    {
        if (gameOverText3D == null || !gameOverText3D.gameObject.activeInHierarchy) 
            return;

        Vector3 directionToCamera = playerCamera.position - gameOverText3D.transform.position;
        directionToCamera.y = 0;
        
        if (directionToCamera != Vector3.zero){
            gameOverText3D.transform.rotation = Quaternion.LookRotation(-directionToCamera);
        }
    }

    void RestartGame()
    {
        Debug.Log("[GhostGameOver] Restarting game - reloading scene...");
        
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }

    void RotateCameraTowardsGhost()
    {
        // target 1 m above the ghost's position
        Vector3 targetPos = ghost.position + Vector3.up * 1f;
        Vector3 dir       = targetPos - playerCamera.position;

        // compute desired look rotation
        Quaternion look   = Quaternion.LookRotation(dir.normalized);

        // slerp toward it
        playerCamera.rotation = Quaternion.Slerp(
            playerCamera.rotation,
            look,
            Time.deltaTime * rotateSpeed
        );
    }
}