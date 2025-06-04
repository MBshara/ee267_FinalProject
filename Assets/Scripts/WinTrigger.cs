using UnityEngine;
using UnityEngine.SceneManagement;
using TMPro;

public class WinTrigger : MonoBehaviour
{
    [Header("References")]
    public Transform playerCamera;        
    public TextMeshPro winText3D;          
    public ReadUSB usbRotationScript; 
    public MonoBehaviour[]  scriptsToDisable;

    [Header("Win Trigger Settings")]
    public Vector3 winZonePosition; 
    public float winZoneRadius = 1f; 
    public bool autoDetectHallwayEnd = true; 
    public float hallwayLength = 10f;
    public int mazeRows = 10;
    public int mazeCols = 10;
    public float hallwayWidth = 1f;
    
    [Header("3D Text Settings")]
    public float textDistanceFromPlayer = 3f;
    public float textHeight = 1.8f;
    public bool textFollowsPlayer = true;
    public float textScale = 0.15f; 
    public string winMessage = "Congratulations! You escaped the Maze!";
    
    [Header("Win Behavior")]
    public bool disableMovementOnWin = false;
    public float celebrationTime = 5f;

    bool hasWon = false;
    float winStartTime;
    Vector3 fixedTextPosition;
    bool celebrationFinished = false;

    void Start()
    {
        // Make sure the win text is hidden at start
        if (winText3D !=null)
            winText3D.gameObject.SetActive(false);
            
        // Auto-detect win zone position if enabled
        if (autoDetectHallwayEnd){
            CalculateWinZonePosition();
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Y) &&(celebrationFinished || !hasWon)) // Done
        {
            RestartGame();
            return;
        }
        
        if (hasWon)
        {
            UpdateTextPosition();
            MakeTextFaceCamera();
            
            if (!celebrationFinished && Time.time - winStartTime > celebrationTime){
                celebrationFinished = true;
                Debug.Log("[WinTrigger] Celebration finished! Press Y to restart.");
            }
            
            return;
        }

        // Check if player has reached the win zone
        Vector3 playerPos = transform.position;
        float distanceToWinZone = Vector3.Distance(playerPos, winZonePosition);
        
        if (distanceToWinZone<= winZoneRadius){
            Debug.Log($"[WinTrigger] Player reached win zone! Distance: {distanceToWinZone:F2}");
            TriggerWin();
        }
    }

    // IGNORE: NOT FUNCTIONAL
    void CalculateWinZonePosition()
    {
        Vector3 goalCellPos = new Vector3((mazeRows -1) * hallwayWidth,0f, (mazeCols - 1) * hallwayWidth);
        winZonePosition = goalCellPos + new Vector3(hallwayWidth* 0.5f, 0f, hallwayWidth + hallwayLength - 1f);
        
        Debug.Log($"[WinTrigger] Win zone: {winZonePosition}");
    }

    void TriggerWin()
    {
        hasWon = true;
        winStartTime = Time.time;
        celebrationFinished = false;

        Debug.Log("[WinTrigger] VICTORY! Player has escaped!");

        if (disableMovementOnWin)
        {
            // I dont think this works...
            if (usbRotationScript != null)
            {
                usbRotationScript.enabled = false;
                Debug.Log("[WinTrigger] Disabled USB rotation script");
            }

            // THIS WORKS: Stop any other movement/input scripts
            if (scriptsToDisable != null){
                foreach (var s in scriptsToDisable){
                    if (s != null) {
                        s.enabled = false;
                        Debug.Log($"[WinTrigger] Disabled script: {s.GetType().Name}");
                    }
                }
            }

            // Turn off ability to look around
            if (TryGetComponent<CharacterController>(out var cc))
            {
                cc.enabled = false;
                Debug.Log("[WinTrigger] Disabled CharacterController");
            }
        }

        Show3DWinText();
    }

    void Show3DWinText()
    {
        if (winText3D != null)
        {
            winText3D.text = winMessage;
            winText3D.gameObject.SetActive(true);
            
            // Set initial position
            if (!textFollowsPlayer)
            {
                // Follow initial looking pos
                Vector3 forward = playerCamera.forward;
                forward.y = 0;
                forward.Normalize();
                
                fixedTextPosition = transform.position + forward * textDistanceFromPlayer;
                fixedTextPosition.y = textHeight;
                winText3D.transform.position = fixedTextPosition;
            }
            
            winText3D.transform.localScale = Vector3.one * textScale;
            
            winText3D.fontSize = 32;
            winText3D.alignment = TextAlignmentOptions.Center;
            winText3D.color = Color.green;
            winText3D.fontMaterial.SetFloat("_OutlineWidth", 0.2f);
            winText3D.fontMaterial.SetColor("_OutlineColor", Color.black);
        }
    }

    void UpdateTextPosition()
    {
        if (winText3D == null || !winText3D.gameObject.activeInHierarchy) 
            return;

        if (textFollowsPlayer){
            Vector3 forward = playerCamera.forward;
            forward.y = 0;
            forward.Normalize();
            
            Vector3 textPos = playerCamera.position + forward * textDistanceFromPlayer;
            textPos.y = textHeight;
            winText3D.transform.position = textPos;
        }
        else{
            winText3D.transform.position = fixedTextPosition;
        }
    }

    void MakeTextFaceCamera(){
        if (winText3D== null || !winText3D.gameObject.activeInHierarchy) 
            return;

        Vector3 directionToCamera =playerCamera.position - winText3D.transform.position;
        directionToCamera.y = 0;
        
        if (directionToCamera !=Vector3.zero){
            winText3D.transform.rotation = Quaternion.LookRotation(-directionToCamera);
        }
    }

    void RestartGame()
    {
        Debug.Log("[WinTrigger] Reloading scene...");
        SceneManager.LoadScene(SceneManager.GetActiveScene().name);
    }
}