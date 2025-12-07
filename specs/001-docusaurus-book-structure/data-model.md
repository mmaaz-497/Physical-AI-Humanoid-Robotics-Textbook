# Data Model: Docusaurus Book Structure

**Feature**: 001-docusaurus-book-structure
**Date**: 2025-12-06

## Overview

This document defines the entities and their relationships for the Physical AI & Humanoid Robotics Docusaurus book structure. While this is a static documentation site (not a traditional database-backed application), the data model defines the conceptual entities that govern the book's organization.

## Entity Definitions

### 1. Category

**Description**: Top-level sidebar grouping containing multiple related chapters

**Attributes**:
- `id` (string): Unique identifier (e.g., "01-introduction", "03-ros2")
- `label` (string): Display name shown in sidebar (e.g., "Introduction to Physical AI & Humanoid Robotics")
- `position` (integer): Order in sidebar (1-7)
- `collapsible` (boolean): Whether category can be collapsed (always `true`)
- `collapsed` (boolean): Default collapsed state (`false` for active category, `true` otherwise)
- `dirName` (string): Directory name in `/docs` (e.g., "01-introduction")
- `chapters` (array): List of Chapter entities belonging to this category

**Validation Rules**:
- Exactly 7 categories must exist (per FR-001)
- Position must be unique and sequential (1-7)
- Label must match specification exactly
- Directory name must use numeric prefix for ordering (01-07)

**Instances** (as defined in spec):

| ID | Label | Position | Chapters | Dir Name |
|----|-------|----------|----------|----------|
| 01-introduction | Introduction to Physical AI & Humanoid Robotics | 1 | 5 | 01-introduction |
| 02-setup-guides | Setup Guides | 2 | 3 | 02-setup-guides |
| 03-ros2 | Module 1: ROS 2 (Weeks 3–5) | 3 | 5 | 03-ros2 |
| 04-digital-twin | Module 2: Digital Twin (Weeks 6–7) | 4 | 5 | 04-digital-twin |
| 05-isaac | Module 3: NVIDIA Isaac (Weeks 8–10) | 5 | 6 | 05-isaac |
| 06-vla-humanoids | Module 4: VLA & Humanoids (Weeks 11–13) | 6 | 7 | 06-vla-humanoids |
| 07-references | References | 7 | 4 | 07-references |

**Total Chapters**: 5 + 3 + 5 + 5 + 6 + 7 + 4 = 36

---

### 2. Chapter

**Description**: Individual content page within a category

**Attributes**:
- `id` (string): Unique slug (e.g., "what-is-physical-ai")
- `title` (string): Full chapter title (e.g., "What Is Physical AI?")
- `sidebar_label` (string): Short label for sidebar (usually same as title)
- `sidebar_position` (integer): Order within category (1-N)
- `description` (string): Brief summary for SEO and previews
- `keywords` (array): SEO keywords
- `category_id` (string): Foreign key to parent Category
- `file_path` (string): Relative path from `/docs` (e.g., "01-introduction/what-is-physical-ai.mdx")
- `content_sections` (array): List of ContentSection entities
- `assets` (array): List of Asset entities referenced by this chapter

**Validation Rules**:
- ID must be unique across all chapters
- File path must match pattern: `{category_dir}/{chapter-slug}.mdx`
- Must contain all required content sections (per FR-010)
- Must use proper heading hierarchy H1 → H2 → H3 (per FR-011)

**Example Instances** (Category 1: Introduction):

| ID | Title | Position | File Path |
|----|-------|----------|-----------|
| what-is-physical-ai | What Is Physical AI? | 1 | 01-introduction/what-is-physical-ai.mdx |
| embodied-intelligence | Embodied Intelligence & Real-World Agents | 2 | 01-introduction/embodied-intelligence.mdx |
| digital-vs-physical-ai | Difference Between Digital AI and Physical AI | 3 | 01-introduction/digital-vs-physical-ai.mdx |
| humanoid-robotics-overview | Overview of Humanoid Robotics Systems | 4 | 01-introduction/humanoid-robotics-overview.mdx |
| physical-ai-architecture | The Physical AI System Architecture | 5 | 01-introduction/physical-ai-architecture.mdx |

---

### 3. MDX File

**Description**: Markdown file with JSX support containing chapter content

**Attributes**:
- `file_path` (string): Absolute path in repository (e.g., "docs/01-introduction/what-is-physical-ai.mdx")
- `front_matter` (object): YAML metadata block
  - `id` (string): Chapter ID
  - `title` (string): Chapter title
  - `sidebar_label` (string): Sidebar display name
  - `sidebar_position` (integer): Order in category
  - `description` (string): SEO description
  - `keywords` (array): SEO keywords
- `content_body` (markdown/MDX): Main chapter content with sections, code blocks, images

**Front Matter Schema**:
```yaml
---
id: string
title: string
sidebar_label: string
sidebar_position: number (1-N)
description: string
keywords: string[]
---
```

**Validation Rules**:
- File must exist at specified path
- Front matter must parse as valid YAML
- All required front matter fields must be present
- Content body must contain valid MDX (no syntax errors)

---

### 4. Content Section

**Description**: Logical division within a chapter (introduction, theory, code examples, diagrams, exercises)

**Attributes**:
- `section_type` (enum): Type of section
  - `introduction` (required)
  - `theory` (required)
  - `practical_implementation` (optional)
  - `code_examples` (optional)
  - `diagrams` (optional)
  - `exercises` (required)
  - `summary` (optional)
  - `further_reading` (optional)
- `heading` (string): Section heading text (e.g., "Introduction", "Theory", "Exercises")
- `heading_level` (integer): HTML heading level (2 for H2, 3 for H3)
- `content` (markdown): Section content
- `order` (integer): Position within chapter (1-N)

**Validation Rules**:
- Each chapter must have at least: introduction, theory, exercises (per constitution Documentation Standards)
- Heading levels must follow hierarchy (H2 for major sections, H3 for subsections per FR-011)
- `introduction` must always be first section (order = 1)

**Standard Section Order**:
1. Introduction (H2)
2. Theory (H2)
3. Practical Implementation (H2) - if applicable
4. Code Examples (H2) - if applicable
5. Exercises (H2)
6. Summary (H2) - optional
7. Further Reading (H2) - optional

---

### 5. Code Block

**Description**: Syntax-highlighted code example within chapter content

**Attributes**:
- `language` (string): Programming language (python, xml, yaml, bash, cpp, javascript, typescript)
- `code_content` (string): Actual code
- `title` (string): Optional code block title
- `highlight_lines` (array): Line numbers to highlight
- `file_reference` (string): Optional path to full code file in `/static/code/`
- `chapter_id` (string): Foreign key to parent Chapter

**Validation Rules**:
- Language must be supported by Prism syntax highlighter
- Code content must not exceed 100KB (per C-008)
- If `file_reference` provided, file must exist in `/static/code/`

**Supported Languages**:
- `python`: ROS 2 nodes, Python scripts
- `xml`: URDF models, SDF files, ROS 2 XML launch files
- `yaml`: ROS 2 YAML configs, parameters
- `bash`: Shell scripts for setup
- `cpp`: Advanced ROS 2 examples (C++)
- `javascript`: Docusaurus config examples
- `typescript`: TypeScript config examples

**Example**:
```python title="simple_publisher.py" {3,8}
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

---

### 6. Asset

**Description**: Static resource referenced by chapters (images, diagrams, code files, data files)

**Attributes**:
- `asset_id` (string): Unique identifier (e.g., "chapter-01-physical-ai-diagram")
- `file_path` (string): Path relative to `/static` (e.g., "img/chapter-01-physical-ai-diagram.png")
- `asset_type` (enum): Type of asset
  - `image`: PNG, JPG, SVG, WebP
  - `code`: Python, XML, YAML, etc. (full files)
  - `data`: URDF, SDF, meshes
- `file_size` (integer): File size in bytes
- `chapter_references` (array): List of chapter IDs that reference this asset
- `alt_text` (string): Accessibility description (for images)

**Validation Rules**:
- Images must be <500KB (per C-008)
- Code files must be <100KB (per C-008)
- File path must follow naming pattern: `chapter-##-descriptive-name.[ext]`
- Alt text required for all images (accessibility compliance per NFR-003)

**Directory Structure**:
```
static/
├── img/                          # Images and diagrams
│   ├── chapter-01-physical-ai-diagram.png
│   ├── chapter-02-system-architecture.png
│   └── ...
├── code/                         # Code examples
│   ├── ros2/
│   │   ├── simple_publisher.py
│   │   ├── simple_subscriber.py
│   │   └── ...
│   ├── gazebo/
│   ├── isaac/
│   └── vla/
└── data/                         # Data files
    ├── urdf/
    │   ├── humanoid_basic.urdf
    │   └── ...
    └── sdf/
```

---

### 7. Sidebar Configuration

**Description**: JSON/JavaScript configuration defining category and chapter structure

**Attributes**:
- `type` (string): Sidebar type ("autogenerated" or "docs")
- `dirName` (string): Root directory for auto-generation (".")
- `category_definitions` (array): List of category configurations
- `category_json_files` (array): List of `_category_.json` files (one per category directory)

**Category JSON Schema** (`_category_.json`):
```json
{
  "label": "Category Display Name",
  "position": 1-7,
  "collapsible": true,
  "collapsed": false,
  "link": null
}
```

**Implementation**:
```javascript
// sidebars.js
module.exports = {
  docsSidebar: [
    {
      type: 'autogenerated',
      dirName: '.',
    },
  ],
};
```

---

## Entity Relationships

### Category ↔ Chapter (One-to-Many)

- One Category contains multiple Chapters
- Each Chapter belongs to exactly one Category
- Relationship enforced by directory structure: chapters in category subdirectory

**Example**:
- Category "01-introduction" → contains 5 Chapters
- Chapter "what-is-physical-ai" → belongs to Category "01-introduction"

---

### Chapter ↔ Content Section (One-to-Many)

- One Chapter contains multiple Content Sections
- Each Content Section belongs to exactly one Chapter
- Sections ordered sequentially within chapter

**Example**:
- Chapter "what-is-physical-ai" → contains 6 sections (intro, theory, code examples, exercises, summary, further reading)

---

### Chapter ↔ Code Block (One-to-Many)

- One Chapter can contain multiple Code Blocks
- Each Code Block belongs to exactly one Chapter
- Code blocks embedded in Content Sections

---

### Chapter ↔ Asset (Many-to-Many)

- One Chapter can reference multiple Assets (images, code files, data files)
- One Asset can be referenced by multiple Chapters (e.g., shared diagram)
- Assets stored in `/static/` and referenced via relative URLs

**Example**:
- Chapter "humanoid-robotics-overview" → references assets:
  - `static/img/chapter-04-humanoid-diagram.png`
  - `static/data/urdf/humanoid_basic.urdf`
- Asset `static/img/physical-ai-workflow.png` → referenced by multiple chapters in intro and VLA modules

---

## Entity Lifecycle

### Category Lifecycle

1. **Creation**: Define in spec, create directory in `/docs/`, add `_category_.json`
2. **Active**: Contains chapters, rendered in sidebar
3. **Update**: Modify `_category_.json` (e.g., change label or collapsed state)
4. **Deletion**: Remove directory (not expected in this project - fixed structure)

---

### Chapter Lifecycle

1. **Creation**: Generate MDX file with front matter, add to category directory
2. **Draft**: Chapter exists but content incomplete
3. **Review**: Content complete, undergoing quality gate validation
4. **Published**: Passes all quality gates, visible in production
5. **Update**: Content revised, re-validate through quality gates
6. **Archived**: Moved to archive directory (not deleted - version history)

---

### Asset Lifecycle

1. **Creation**: Add file to `/static/img|code|data/`
2. **Referenced**: Linked from one or more chapters
3. **Optimized**: Compressed if exceeds file size limits
4. **Orphaned**: No longer referenced by any chapter (candidate for cleanup)
5. **Deleted**: Removed from `/static/` (only if confirmed orphaned)

---

## Data Integrity Constraints

### Uniqueness Constraints

- **Category.id**: Must be unique across all categories (7 total)
- **Category.position**: Must be unique and sequential (1-7)
- **Chapter.id**: Must be unique across all chapters (36 total)
- **Chapter.file_path**: Must be unique across all MDX files
- **Asset.file_path**: Must be unique across all assets

### Referential Integrity

- **Chapter.category_id** → Must reference valid Category.id
- **CodeBlock.file_reference** → If provided, file must exist in `/static/code/`
- **Asset references in MDX** → All image/code/data references must resolve to existing files

### Cardinality Constraints

- **Categories**: Exactly 7 (per FR-001 through FR-008)
- **Chapters per Category**: Fixed counts (5, 3, 5, 5, 6, 7, 4 per spec)
- **Total Chapters**: Exactly 36

---

## Data Validation Checklist

### Pre-Deployment Validation

- [ ] All 7 categories exist with correct labels and positions
- [ ] All 36 chapters exist with valid front matter
- [ ] All MDX files parse without syntax errors
- [ ] All asset references resolve to existing files
- [ ] All images <500KB, all code files <100KB
- [ ] All chapters contain required sections (intro, theory, exercises)
- [ ] Sidebar renders correctly with collapsible categories
- [ ] No broken links between chapters
- [ ] All code blocks have valid language identifiers

---

## Next Steps

With the data model defined, proceed to:
1. Generate `quickstart.md` with manual testing scenarios
2. Run `/sp.tasks` to create implementation tasks
3. Implement Docusaurus configuration files (docusaurus.config.js, sidebars.js)
4. Create category directories with `_category_.json` files
5. Generate MDX chapter templates
