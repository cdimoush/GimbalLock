# Onshape Robot Model Generator Specification

## Overview
A Python script (`generate.py`) that generates robot models (URDF/MJCF/SDF + STLs) from Onshape assemblies without requiring a config file. The script takes an Onshape URL and extracts all necessary information to generate a complete robot model.

## API Setup & Authentication

### Environment Variables via .env File (DevContainer Workflow)

The project uses a `.env` file in the workspace root to manage Onshape API credentials. The DevContainer automatically sources these variables at container startup.

#### Setup Steps

1. **Get API Keys**
   - Visit https://dev-portal.onshape.com/keys
   - Sign in with your Onshape account
   - Create a new API key pair (Access Key + Secret Key)

2. **Create .env File**
   - Copy `.env.example` to `.env` in the workspace root
   - Fill in your credentials:
   ```bash
   # Onshape API Credentials
   ONSHAPE_API=https://cad.onshape.com
   ONSHAPE_ACCESS_KEY=your_access_key_here
   ONSHAPE_SECRET_KEY=your_secret_key_here
   ```

3. **DevContainer Configuration**
   - The `.devcontainer/devcontainer.json` includes:
   ```json
   "runArgs": ["--env-file", "${localWorkspaceFolder}/.env"]
   ```
   - This automatically loads environment variables from `.env` when the container starts
   - No manual export or sourcing required

4. **Security**
   - `.env` is git-ignored (added to `.gitignore`)
   - Never commit API keys to version control
   - `.env.example` serves as a template (without real credentials)

#### Required Environment Variables
```bash
ONSHAPE_API=https://cad.onshape.com
ONSHAPE_ACCESS_KEY=<your_access_key>
ONSHAPE_SECRET_KEY=<your_secret_key>
```

#### File Structure
```
/workspace/
├── .env                    # Your actual credentials (git-ignored)
├── .env.example            # Template for credentials
├── .gitignore              # Contains .env
├── .devcontainer/
│   ├── devcontainer.json   # Configured with runArgs for .env
│   └── Dockerfile
└── model/
    └── generate.py         # Reads env vars automatically
```

## How onshape-to-robot Works

### Architecture Overview
1. **Config**: Parses configuration and extracts document/workspace/element IDs from URL
2. **Client**: Handles Onshape API authentication using HMAC-SHA256 signatures
3. **Assembly**: Fetches assembly structure, mates, parts, and features from Onshape API
4. **RobotBuilder**: Builds robot structure by:
   - Traversing assembly tree to identify links and joints
   - Downloading STL meshes for each part
   - Extracting mass properties, colors, and dynamics
   - Creating a Robot object with Links, Joints, and Parts
5. **Exporter**: Converts Robot object to output format (URDF/SDF/MuJoCo XML)

### URL Parsing
Onshape URLs follow this pattern:
```
https://{domain}/{type}/d/{document_id}/{w|v}/{workspace_id|version_id}/e/{element_id}
```

Example:
```
https://cad.onshape.com/documents/d/abc123/w/def456/e/ghi789
```

Extracts:
- `document_id`: abc123
- `workspace_id`: def456 (if URL has `/w/`)
- `version_id`: def456 (if URL has `/v/`)
- `element_id`: ghi789

### Key Configuration Parameters Used Internally
From analyzing `Config` class, these parameters are used:
- `document_id`, `workspace_id/version_id`, `element_id`: From URL
- `robot_name`: Derived from folder name or set to "gyro"
- `output_format`: Set to "urdf" and "mujoco" for dual export
- `output_filename`: "robot" (default)
- `assets_directory`: "assets" (for STL files)
- `configuration`: "default" (Onshape configuration string)
- `ignore_limits`: False
- `no_dynamics`: False
- `draw_frames`: False
- `joint_properties`: {} (joint-specific settings)
- `ignore`: {} (parts to ignore)
- `processors`: Uses default processors if not specified
- `post_import_commands`: [] (commands to run after export)

## Script Design: `generate.py`

### Command-Line Interface
```bash
python model/generate.py <onshape_url>
```

### Arguments
- `onshape_url` (required): Full Onshape assembly URL

**Note:** The script always generates both URDF and MuJoCo (MJCF) formats. No format selection needed.

### Output Structure
```
model/gyro/
├── urdf/
│   └── robot.urdf
├── mjcf/
│   └── robot.xml
└── assets/
    ├── part1.stl
    ├── part1.part      # Metadata JSON
    ├── part2.stl
    ├── part2.part
    └── ...
```

### Implementation Approach

The script will:

1. **Parse Command Line Arguments**
   - Extract Onshape URL (only required argument)

2. **Extract IDs from URL**
   - Use regex similar to `Config.parse_url()` method
   - Pattern: `https://(.*)/(.*)/([wv])/(.*)/e/(.*)`
   - Extract: document_id, workspace_id OR version_id, element_id

3. **Create Output Directories**
   - Create `model/gyro/urdf/` directory
   - Create `model/gyro/mjcf/` directory
   - Create `model/gyro/assets/` directory (shared by both formats)

4. **Build Robot Model Once**
   - Create a temporary dummy config file for onshape-to-robot compatibility
   - Instantiate `RobotBuilder(config)` which:
     - Creates `Assembly` object (fetches from Onshape API)
     - Traverses assembly tree
     - Downloads STL files to `model/gyro/assets/`
     - Extracts dynamics (mass, COM, inertia)
   - Get the built `robot` object (used for both exports)

5. **Apply Processors**
   - Use default processors from `onshape_to_robot.processors`
   - Processors handle tasks like:
     - Fixed link merging
     - Dummy base links
     - Collision mesh simplification

6. **Export to Both Formats**
   - **URDF Export:**
     - Instantiate `ExporterURDF(config)`
     - Call `exporter.write_xml(robot, "model/gyro/urdf/robot.urdf")`
   - **MuJoCo Export:**
     - Instantiate `ExporterMuJoCo(config)`
     - Call `exporter.write_xml(robot, "model/gyro/mjcf/robot.xml")`

**Key Simplification:** The robot model is built once and exported to both formats, eliminating the need for format selection.

### Differences from Original `export.py`

| Original | generate.py |
|----------|-------------|
| Requires config.json file | No config file needed (temporary one created internally) |
| User provides robot directory | Hardcoded to `model/gyro/` |
| Robot name from config/directory | Hardcoded to "gyro" |
| Reads all settings from config | Uses sensible defaults |
| Single output format per run | Always generates both URDF and MuJoCo |
| User selects format via config | No format selection needed |

## Development Tasks

### Phase 0: Environment Setup (Prerequisites)
1. Create `.gitignore` file with `.env` entry
2. Create `.env.example` template file
3. Update `.devcontainer/devcontainer.json` with `runArgs` for .env loading
4. Rebuild DevContainer to apply changes

### Phase 1: Core Implementation
1. Create URL parser function
2. Create temporary config file generator for API compatibility
3. Implement main script logic with single URL argument
4. Build robot model once
5. Export to both URDF and MuJoCo formats
6. Test both outputs

### Phase 2: Error Handling & Polish
1. Add error handling for:
   - Invalid URLs
   - Missing API credentials
   - API errors
   - Network issues
2. Add helpful error messages
3. Add progress indicators

## Testing

### Prerequisites
- Valid Onshape account
- API keys configured
- Example Onshape assembly URL

### Test Cases
1. Generate both URDF and MuJoCo from a single URL
2. Verify both `model/gyro/urdf/robot.urdf` and `model/gyro/mjcf/robot.xml` created
3. Verify shared `model/gyro/assets/` contains STL files and metadata
4. Test with missing credentials (should show helpful error)
5. Test with invalid URL (should show helpful error)
6. Verify both formats reference the same STL meshes in assets/
7. Verify both outputs are valid (can be loaded by respective simulators)

## Key Dependencies
- `onshape-to-robot` package (already installed)
- Environment variables from `.env` file for API auth
- Internet connection for API calls

## Notes
- The script will create `model/gyro/` directory if it doesn't exist
- STL files and metadata stored in `model/gyro/assets/`
- The onshape-to-robot library handles all API authentication via HMAC-SHA256
- Caching is built into the library for repeated API calls
- Default processors are sensible for most robot models
- Environment variables are automatically loaded by DevContainer at startup via `--env-file` flag