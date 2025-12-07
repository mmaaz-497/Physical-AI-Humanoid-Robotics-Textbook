# Quickstart Guide: Docusaurus Book Structure

**Feature**: 001-docusaurus-book-structure
**Date**: 2025-12-06
**Purpose**: Manual testing scenarios for validating the MVP and full implementation

## Overview

This quickstart guide provides step-by-step testing scenarios to validate the Docusaurus book structure at different stages of implementation. Follow these scenarios to ensure the site meets all functional and non-functional requirements.

---

## Prerequisites

Before testing, ensure you have:

- **Node.js**: Version 18.x or 20.x LTS installed
- **npm**: Version 8.x or higher
- **Git**: For cloning the repository
- **Modern Browser**: Chrome 90+, Firefox 88+, Safari 14+, or Edge 90+

**Installation Verification**:
```bash
node --version  # Should show v18.x or v20.x
npm --version   # Should show 8.x or higher
```

---

## Stage 1: MVP Testing (User Story P1)

**Goal**: Verify sidebar navigation with 7 collapsible categories and basic chapter structure

### Scenario 1.1: Install and Build Docusaurus Site

**Steps**:
1. Navigate to project root directory
2. Install dependencies:
   ```bash
   npm install
   ```
3. Start development server:
   ```bash
   npm start
   ```
4. Open browser to `http://localhost:3000`

**Expected Results**:
- ✅ Site loads within 3 seconds (SC-001)
- ✅ No build errors in console
- ✅ Home page renders correctly

---

### Scenario 1.2: Verify Sidebar Structure

**Steps**:
1. With dev server running, examine the left sidebar
2. Count the number of top-level categories
3. Verify category labels match specification
4. Click each category arrow to expand/collapse

**Expected Results**:
- ✅ Exactly 7 categories visible (FR-001)
- ✅ Categories in correct order (FR-002 through FR-008):
  1. Introduction to Physical AI & Humanoid Robotics
  2. Setup Guides
  3. Module 1: ROS 2 (Weeks 3–5)
  4. Module 2: Digital Twin (Weeks 6–7)
  5. Module 3: NVIDIA Isaac (Weeks 8–10)
  6. Module 4: VLA & Humanoids (Weeks 11–13)
  7. References
- ✅ Each category is collapsible (FR-013)
- ✅ Categories expand/collapse on click
- ✅ Sidebar responds within 100ms (NFR-002)

---

### Scenario 1.3: Verify Chapter Count per Category

**Steps**:
1. Expand each category one at a time
2. Count the number of child chapters in each category

**Expected Results**:
- ✅ Category 1 (Introduction): 5 chapters (FR-002)
- ✅ Category 2 (Setup Guides): 3 chapters (FR-003)
- ✅ Category 3 (ROS 2): 5 chapters (FR-004)
- ✅ Category 4 (Digital Twin): 5 chapters (FR-005)
- ✅ Category 5 (NVIDIA Isaac): 6 chapters (FR-006)
- ✅ Category 6 (VLA & Humanoids): 7 chapters (FR-007)
- ✅ Category 7 (References): 4 chapters (FR-008)
- ✅ **Total**: 36 chapters

---

### Scenario 1.4: Test Chapter Navigation

**Steps**:
1. Click "What Is Physical AI?" in the Introduction category
2. Verify the chapter content loads
3. Check that the active chapter is highlighted in sidebar
4. Click another chapter in a different category
5. Verify navigation works correctly

**Expected Results**:
- ✅ Chapter content displays (FR-016)
- ✅ Active chapter highlighted in sidebar (FR-014)
- ✅ URL updates to reflect current chapter
- ✅ No broken links (SC-003)

---

### Scenario 1.5: Test Mobile Responsiveness

**Steps**:
1. Open browser dev tools (F12)
2. Switch to device emulation mode
3. Test three viewport sizes:
   - Desktop: 1920px width
   - Tablet: 768px width
   - Mobile: 375px width
4. Verify sidebar behavior at each size

**Expected Results**:
- ✅ Desktop (1920px): Sidebar always visible (SC-004)
- ✅ Tablet (768px): Sidebar collapsible via hamburger menu (SC-004)
- ✅ Mobile (375px): Sidebar full-screen overlay (SC-004)
- ✅ All viewports: Content readable and navigable (NFR-003)

---

## Stage 2: Content Validation (User Stories P2-P8)

**Goal**: Verify chapter structure, content quality, and constitution compliance

### Scenario 2.1: Validate Chapter Structure

**Steps**:
1. Open any chapter (e.g., "What Is Physical AI?")
2. Verify presence of required sections:
   - Introduction
   - Theory
   - Exercises
3. Check optional sections (if applicable):
   - Practical Implementation
   - Code Examples
   - Diagrams
   - Summary
   - Further Reading

**Expected Results**:
- ✅ All chapters follow consistent structure (FR-010)
- ✅ Heading hierarchy: H1 (title) → H2 (sections) → H3 (subsections) (FR-011)
- ✅ Introduction section appears first
- ✅ Exercises section present
- ✅ Content understandable by intermediate students (SC-008)

---

### Scenario 2.2: Validate Code Syntax Highlighting

**Steps**:
1. Navigate to a chapter with code examples (e.g., "ROS 2 Nodes and Topics")
2. Verify code blocks are syntax-highlighted
3. Test multiple language types:
   - Python
   - XML (URDF)
   - YAML
   - Bash

**Expected Results**:
- ✅ Syntax highlighting active (FR-015)
- ✅ Keywords, strings, comments colored appropriately
- ✅ Copy button appears on code blocks (NFR-006)
- ✅ Code examples are functional and reproducible (SC-005)

---

### Scenario 2.3: Validate Assets (Images, Code Files)

**Steps**:
1. Navigate to chapters with diagrams (e.g., "Physical AI System Architecture")
2. Verify images load correctly
3. Check image file sizes (should be <500KB per C-008)
4. Verify code file links in `/static/code/`

**Expected Results**:
- ✅ All images load without errors (FR-018)
- ✅ Images have descriptive filenames (e.g., `chapter-01-physical-ai-diagram.png`)
- ✅ Images <500KB (C-008)
- ✅ Code files <100KB (C-008)
- ✅ Images have alt text for accessibility (NFR-003)

---

### Scenario 2.4: Test Search Functionality

**Steps**:
1. Click the search icon (usually top-right)
2. Search for a technical term (e.g., "ROS 2")
3. Verify search results appear
4. Measure response time

**Expected Results**:
- ✅ Search results appear within 2 seconds (SC-006)
- ✅ Results are relevant to the query
- ✅ Clicking a result navigates to the correct chapter

---

### Scenario 2.5: Test Dark Mode

**Steps**:
1. Locate the theme toggle (sun/moon icon)
2. Switch between light and dark modes
3. Verify all pages render correctly in both modes

**Expected Results**:
- ✅ Dark mode toggle present (NFR-009)
- ✅ All content readable in both themes
- ✅ Code syntax highlighting works in both modes
- ✅ No color contrast issues (WCAG 2.1 AA per NFR-003)

---

## Stage 3: Build and Deployment Testing

**Goal**: Verify production build and GitHub Pages deployment

### Scenario 3.1: Production Build

**Steps**:
1. Stop dev server
2. Build for production:
   ```bash
   npm run build
   ```
3. Verify build completes without errors
4. Serve production build locally:
   ```bash
   npm run serve
   ```
5. Test production site at `http://localhost:3000`

**Expected Results**:
- ✅ Build completes with zero errors (SC-009)
- ✅ Production site loads within 3 seconds (NFR-001)
- ✅ All features work identically to dev mode

---

### Scenario 3.2: GitHub Pages Deployment

**Steps**:
1. Verify GitHub Actions workflow exists at `.github/workflows/deploy.yml`
2. Push changes to `main` branch
3. Monitor GitHub Actions run
4. Visit deployed site at `https://<username>.github.io/<repo-name>/`

**Expected Results**:
- ✅ GitHub Actions workflow runs successfully
- ✅ Site deploys to GitHub Pages (NFR-007)
- ✅ Deployed site accessible via URL
- ✅ All features work on deployed site

---

## Stage 4: Constitution Compliance Testing

**Goal**: Validate adherence to all seven constitution principles

### Scenario 4.1: Accuracy Validation

**Steps**:
1. Review technical content in sample chapters (ROS 2, Isaac, Gazebo)
2. Cross-reference code examples with official documentation
3. Verify version numbers are documented

**Expected Results**:
- ✅ All technical content references official docs (Principle I)
- ✅ Code examples tested on Ubuntu 22.04 (Principle IV)
- ✅ Version numbers explicitly documented

---

### Scenario 4.2: Clarity Validation

**Steps**:
1. Have an intermediate AI/robotics student review 3-5 chapters
2. Collect feedback on comprehensibility
3. Verify learning objectives are met

**Expected Results**:
- ✅ Content understandable by target audience (Principle II)
- ✅ Complex concepts broken into digestible segments
- ✅ Clear progression from theory to practice

---

### Scenario 4.3: Hierarchical Organization Validation

**Steps**:
1. Review chapter structure across all 36 chapters
2. Verify heading hierarchy (H1 → H2 → H3)
3. Check logical flow from intro → theory → practice → exercises

**Expected Results**:
- ✅ Consistent structure across all chapters (Principle III)
- ✅ Proper heading hierarchy maintained
- ✅ Logical content progression

---

### Scenario 4.4: Reproducibility Validation

**Steps**:
1. Set up a fresh Ubuntu 22.04 environment
2. Follow setup guides (Digital Twin Workstation)
3. Execute code examples from ROS 2 chapters
4. Verify all examples run successfully

**Expected Results**:
- ✅ All tutorials reproducible on Ubuntu 22.04 (Principle IV)
- ✅ Dependencies documented with versions
- ✅ 100% success rate for code examples (SC-005)

---

### Scenario 4.5: Consistency Validation

**Steps**:
1. Review writing tone across 10 randomly selected chapters
2. Check code formatting (Python PEP 8, ROS 2 style guides)
3. Verify technical terms defined consistently

**Expected Results**:
- ✅ Unified writing tone (technical, clear, step-by-step) (Principle VI)
- ✅ Code formatting follows conventions
- ✅ Glossary terms used consistently

---

## Success Criteria Checklist

Use this checklist to validate all success criteria from spec.md:

- [ ] **SC-001**: Site renders with all 7 categories and 36 chapters in <3 seconds
- [ ] **SC-002**: 100% of chapters follow defined structure
- [ ] **SC-003**: Zero broken links across all chapters
- [ ] **SC-004**: Sidebar functional on desktop/tablet/mobile (1920px/768px/375px)
- [ ] **SC-005**: All code examples execute on Ubuntu 22.04 (100% reproducibility)
- [ ] **SC-006**: Search returns results in <2 seconds
- [ ] **SC-007**: Book supports 13-week curriculum progression
- [ ] **SC-008**: 95% of students complete first module successfully
- [ ] **SC-009**: Site deploys to GitHub Pages with zero build errors
- [ ] **SC-010**: All chapters pass constitution quality gates

---

## Troubleshooting

### Issue: Sidebar categories not collapsible

**Solution**: Verify `_category_.json` files exist in each category directory with `"collapsible": true`

---

### Issue: Code blocks not syntax-highlighted

**Solution**: Check `docusaurus.config.js` includes language in `prism.additionalLanguages` array

---

### Issue: Images not loading

**Solution**: Verify image paths are relative to `/static/` and use correct syntax: `![Alt text](/img/filename.png)`

---

### Issue: Search not working

**Solution**: Ensure Docusaurus search plugin is installed and configured in `docusaurus.config.js`

---

### Issue: Build fails on GitHub Actions

**Solution**: Check Node.js version in workflow file matches local development version (18.x or 20.x)

---

## Next Steps

After completing all testing scenarios:

1. **Document Results**: Record pass/fail status for each scenario
2. **Address Failures**: Fix any issues found during testing
3. **Repeat Testing**: Re-run failed scenarios after fixes
4. **Quality Gate Review**: Validate against all constitution quality gates
5. **Deployment**: Deploy to production if all tests pass
6. **User Acceptance**: Have target audience (intermediate AI/robotics students) validate content

---

## Testing Frequency

- **Local Development**: Run Scenarios 1.1-1.4 after every significant change
- **Before PR Merge**: Run all Stage 1 and Stage 2 scenarios
- **Before Production Deploy**: Run all stages (1-4) plus constitution compliance
- **Periodic Review**: Monthly checks for SC-005 (code reproducibility) as tool versions update

---

## Appendix: Manual Testing Commands

```bash
# Install dependencies
npm install

# Start dev server
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Check for broken links (if link checker installed)
npm run check:links

# Validate MDX syntax
npm run lint:mdx

# Check file sizes
find static/img -type f -size +500k  # Images >500KB
find static/code -type f -size +100k  # Code files >100KB
```

---

**End of Quickstart Guide**
