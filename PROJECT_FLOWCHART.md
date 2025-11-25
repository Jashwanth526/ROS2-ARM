# ArduinoBot Project - Complete System Flowchart

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'fontSize':'16px'}}}%%
flowchart LR
    Start([User: Scan Blue]) --> ScanBlock["Scan & Detect"]
    
    ScanBlock --> N1[Found!]
    ScanBlock -.->|No object| Fail[Nothing Found]
    
    N1 --> Pick([User: Pick])
    Pick --> PickBlock["Pick Sequence"]
    
    PickBlock --> N2[Picked!]
    
    N2 --> Place([User: Place])
    Place --> PlaceBlock["Place Sequence"]
    
    PlaceBlock --> N3[Done!]
    N3 --> End([Complete])
    Fail --> End
    
    subgraph ScanBlock["Scan & Detect"]
        direction TB
        S1[Alexa] --> S2[Bridge]
        S2 --> S3[Task Server]
        S3 --> S4[Scan]
        S4 --> S5[Rotate + Camera]
        S5 --> S6[Vision]
        S6 --> S7{Found?}
        S7 -.->|No| S4
        S7 -->|Yes| S8[Save XYZ]
    end
    
    subgraph PickBlock["Pick Sequence"]
        direction TB
        P1[Plan Path] --> P2[Move Above]
        P2 --> P3[Open Gripper]
        P3 --> P4[Lower Down]
        P4 --> P5[Close Gripper]
        P5 --> P6[Lift Up]
        P6 --> P7[Return Home]
    end
    
    subgraph PlaceBlock["Place Sequence"]
        direction TB
        PL1[Plan Path] --> PL2[Move to Drop Zone]
        PL2 --> PL3[Open Gripper]
        PL3 --> PL4[Release Object]
        PL4 --> PL5[Return Home]
    end
```

---

## How to View This Diagram

### In VS Code:
1. Install "Markdown Preview Mermaid Support" extension
2. Open this file and press `Ctrl+Shift+V` (preview)

### Online:
Copy the diagram and paste at: https://mermaid.live/

### Export as Image:
Use Mermaid Live Editor to export as PNG/SVG

---

## Simple Explanation

**What this shows:** The complete journey from voice command to finished task.

1. **User speaks** to Alexa ("Scan for blue")
2. **Alexa** sends command through Flask bridge
3. **Task Server** (the brain) coordinates everything
4. **Camera** looks around while arm rotates
5. **Vision** finds the blue object and saves location
6. **Alexa** confirms "Found blue!"
7. **User** says "Pick it"
8. **MoveIt** plans safe path to object
9. **Robot** moves, grabs, lifts, returns home
10. **User** says "Place it"
11. **Robot** moves to drop zone and releases
12. **Done!**

**Key Parts:**
- **Rounded boxes** = User interaction
- **Rectangle boxes** = System processes
- **Diamond shapes** = Decision points

