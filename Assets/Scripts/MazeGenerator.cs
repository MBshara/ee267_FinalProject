using System.Collections.Generic;
using UnityEngine;

public class MazeGenerator : MonoBehaviour
{
    [Header("Maze Dims")]
    public int rows = 10;
    public int cols = 10;

    [Header("Seed Settings")]
    public bool useSeed = false;
    public int seed = 12345;
    
    [Header("Geo")]
    public float hallwayWidth = 1f;
    public float wallHeight = 3f;
    public float wallThickness = 0.1f;
    
    [Header("Exit Hallway")]
    public float exitHallwayLength = 10f; 
    public bool createEndWall = true;

    [Header("Materials (optional)")]
    public Material wallMaterial;
    public Material floorMaterial;
    public Material exitHallwayMaterial;
    // public Material endMaterial;

    private int[,] cells;

    private readonly int[] dx = { 0, 1, 0, -1 }; // row
    private readonly int[] dz = { 1, 0, -1, 0 }; // col

    void Start(){
        GeneratePerfectMaze();
        KnockOutGoalNorthWall(); 
        BuildGeometry();
        BuildExitHallway();
    }

    #region Perfect‑maze generation (recursive back‑tracker)
    void GeneratePerfectMaze()
    {
        cells = new int[rows, cols];
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                cells[r, c] = 0b1111; // four walls up

        bool[,] visited = new bool[rows, cols];
        Stack<Vector2Int> stack = new Stack<Vector2Int>();

        visited[0, 0] = true;
        stack.Push(Vector2Int.zero);
        
        System.Random rng;
        if (useSeed){
            rng = new System.Random(seed);
            Debug.Log($"Generating maze with seed: {seed}");
        }
        else{
            rng = new System.Random();
            Debug.Log("Generating maze with random seed");
        }

        while (stack.Count > 0)
        {
            Vector2Int cell = stack.Peek();

            // gather unvisited neighbours
            List<int> neighbours = new List<int>();
            for (int dir = 0; dir < 4; dir++){
                int nr = cell.x + dx[dir];
                int nc = cell.y + dz[dir];
                if (nr >= 0 && nc >= 0 && nr < rows && nc < cols && !visited[nr, nc])
                    neighbours.Add(dir);
            }

            if (neighbours.Count == 0) { stack.Pop(); continue; } // dead end

            int dirSel = neighbours[rng.Next(neighbours.Count)];
            int nrSel  = cell.x + dx[dirSel];
            int ncSel  = cell.y + dz[dirSel];

            // knock down wall between cell and neighbour
            cells[cell.x, cell.y] &= ~(1 << dirSel);
            cells[nrSel, ncSel]  &= ~(1 << ((dirSel + 2) % 4));

            visited[nrSel, ncSel] = true;
            stack.Push(new Vector2Int(nrSel, ncSel));
        }
    }
    #endregion

    #region Opening end wall
    void KnockOutGoalNorthWall()
    {
        // removes escape room wall
        if (rows > 0 && cols > 0)
            cells[rows - 1, cols - 1] &= ~1; 
    }
    #endregion

    #region Geoms
    void BuildGeometry()
    {
        Transform parent = transform;
        float half = hallwayWidth * 0.5f;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                Vector3 origin = new Vector3(r * hallwayWidth, 0f, c * hallwayWidth);

                // Floor tile
                GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
                floor.transform.SetParent(parent);
                floor.transform.localScale = new Vector3(hallwayWidth, 0.05f, hallwayWidth);
                floor.transform.position   = origin + new Vector3(half, -0.025f, half);
                if (floorMaterial) floor.GetComponent<Renderer>().material = floorMaterial;

                // goal marker at (rows‑1, cols‑1)
                // if (r == rows - 1 && c == cols - 1)
                // {
                //     GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                //     marker.transform.SetParent(parent);
                //     float radius = hallwayWidth * 0.3f;
                //     marker.transform.localScale = new Vector3(radius, wallHeight * 0.5f, radius);
                //     marker.transform.position   = origin + new Vector3(half, wallHeight * 0.5f, half);
                //     var mat = marker.GetComponent<Renderer>();
                //     // if (endMaterial) mat.material = endMaterial;
                //     // else             mat.material.color = Color.green;
                // }

                // Walls according to mask
                int mask = cells[r, c];
                for (int dir = 0; dir < 4; dir++){
                    if ((mask & (1 << dir)) == 0) continue;

                    GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    wall.transform.SetParent(parent);

                    Vector3 scale, pos;
                    switch (dir){
                        case 0: // north (+Z)
                            scale = new Vector3(hallwayWidth, wallHeight, wallThickness);
                            pos   = origin + new Vector3(half, wallHeight * 0.5f, hallwayWidth);
                            break;
                        case 1: // east (+X)
                            scale = new Vector3(wallThickness, wallHeight, hallwayWidth);
                            pos   = origin + new Vector3(hallwayWidth, wallHeight * 0.5f, half);
                            break;
                        case 2: // south (‑Z)
                            scale = new Vector3(hallwayWidth, wallHeight, wallThickness);
                            pos   = origin + new Vector3(half, wallHeight * 0.5f, 0f);
                            break;
                        default: // west (‑X)
                            scale = new Vector3(wallThickness, wallHeight, hallwayWidth);
                            pos   = origin + new Vector3(0f, wallHeight * 0.5f, half);
                            break;
                    }

                    wall.transform.localScale = scale;
                    wall.transform.position   = pos;
                    wall.isStatic = true;
                    if (wallMaterial) wall.GetComponent<Renderer>().material = wallMaterial;
                }
            }
        }
    }

    void BuildExitHallway()
    {
        if (exitHallwayLength <= 0) return;

        Transform parent = transform;
        float half = hallwayWidth * 0.5f;
        Vector3 goalCellOrigin = new Vector3((rows - 1) * hallwayWidth, 0f, (cols - 1) * hallwayWidth);
        int numFloorSegments = Mathf.CeilToInt(exitHallwayLength / hallwayWidth);
        
        for (int i = 0; i < numFloorSegments; i++){
            float segmentZ = (cols - 1) * hallwayWidth + (i + 1) * hallwayWidth;
            Vector3 floorOrigin = new Vector3((rows - 1) * hallwayWidth, 0f, segmentZ);
            
            // create floor segment
            GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.transform.SetParent(parent);
            floor.transform.localScale = new Vector3(hallwayWidth, 0.05f, hallwayWidth);
            floor.transform.position = floorOrigin + new Vector3(half, -0.025f, half);
            
            if (exitHallwayMaterial) 
                floor.GetComponent<Renderer>().material = exitHallwayMaterial;
            else if (floorMaterial) 
                floor.GetComponent<Renderer>().material = floorMaterial;
        }
        
        // create side walls for the entire hallway length
        // Left wall
        GameObject leftWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        leftWall.transform.SetParent(parent);
        leftWall.transform.localScale = new Vector3(wallThickness, wallHeight, exitHallwayLength);
        leftWall.transform.position = goalCellOrigin + new Vector3(0f, wallHeight * 0.5f, hallwayWidth + exitHallwayLength * 0.5f);
        leftWall.isStatic = true;
        if (wallMaterial) leftWall.GetComponent<Renderer>().material = wallMaterial;
        
        // Right wall
        GameObject rightWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        rightWall.transform.SetParent(parent);
        rightWall.transform.localScale = new Vector3(wallThickness, wallHeight, exitHallwayLength);
        rightWall.transform.position = goalCellOrigin + new Vector3(hallwayWidth, wallHeight * 0.5f, hallwayWidth + exitHallwayLength * 0.5f);
        rightWall.isStatic = true;
        if (wallMaterial) rightWall.GetComponent<Renderer>().material = wallMaterial;
        
        // End wall (North side at the end of hallway)
        if (createEndWall)
        {
            GameObject endWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
            endWall.transform.SetParent(parent);
            endWall.transform.localScale = new Vector3(hallwayWidth, wallHeight, wallThickness);
            endWall.transform.position = goalCellOrigin + new Vector3(half, wallHeight * 0.5f, hallwayWidth + exitHallwayLength);
            endWall.isStatic = true;
            if (wallMaterial) endWall.GetComponent<Renderer>().material = wallMaterial;
        }
        
        Debug.Log($"Exit hallway built: {exitHallwayLength}m long, {numFloorSegments} floor segments, end wall: {createEndWall}");
    }
    #endregion

    // #region Utility Methods
    // /// <summary>
    // /// Regenerate the maze with current settings (useful for testing different seeds)
    // /// Call this from a button or script to regenerate without stopping/starting play mode
    // /// </summary>
    // [ContextMenu("Regenerate Maze")]
    // public void RegenerateMaze()
    // {
    //     // Clear existing maze
    //     for (int i = transform.childCount - 1; i >= 0; i--)
    //     {
    //         if (Application.isPlaying)
    //             Destroy(transform.GetChild(i).gameObject);
    //         else
    //             DestroyImmediate(transform.GetChild(i).gameObject);
    //     }
        
    //     // Generate new maze
    //     GeneratePerfectMaze();
    //     KnockOutGoalNorthWall();
    //     BuildGeometry();
    //     BuildExitHallway();
    // }
    // #endregion
}