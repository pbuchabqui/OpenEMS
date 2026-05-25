```markdown
# OpenEMS Development Patterns

> Auto-generated skill from repository analysis

## Overview
This skill teaches the core development patterns and conventions used in the OpenEMS TypeScript codebase. It covers file naming, import/export styles, commit message patterns, and testing approaches. By following these guidelines, contributors can maintain consistency and quality across the project.

## Coding Conventions

### File Naming
- Use **snake_case** for all file names.
  - Example: `energy_manager.ts`, `device_controller.test.ts`

### Import Style
- Use **relative imports** for referencing other modules.
  - Example:
    ```typescript
    import { calculatePower } from './power_utils';
    ```

### Export Style
- Use **named exports** for all modules.
  - Example:
    ```typescript
    // In device_controller.ts
    export function startDevice() { ... }
    export function stopDevice() { ... }
    ```

### Commit Message Patterns
- Commit messages are freeform but often start with a descriptive title.
- Average commit message length: ~62 characters.
  - Example:  
    ```
    Add battery management logic to energy_manager
    ```

## Workflows

### Adding a New Module
**Trigger:** When creating a new functional module in the codebase  
**Command:** `/add-module`

1. Create a new file using snake_case (e.g., `new_feature.ts`).
2. Implement your logic using named exports.
3. Use relative imports to include dependencies.
4. Write corresponding tests in a file named `new_feature.test.ts`.
5. Commit with a descriptive message.

### Writing Tests
**Trigger:** When adding or updating functionality  
**Command:** `/write-tests`

1. Create a test file with the pattern `*.test.ts` (e.g., `device_controller.test.ts`).
2. Write tests for all exported functions.
3. Use the project's preferred (unknown) test framework.
4. Run tests to ensure correctness.

### Importing and Exporting Functions
**Trigger:** When sharing logic between modules  
**Command:** `/import-export`

1. Use named exports in your module:
    ```typescript
    export function calculateEfficiency() { ... }
    ```
2. Import using a relative path in another module:
    ```typescript
    import { calculateEfficiency } from './efficiency_utils';
    ```

## Testing Patterns

- Test files follow the `*.test.ts` naming convention.
- Each test file should correspond to a source file.
- The testing framework is not explicitly specified; check existing test files for patterns.
- Example test file:
    ```typescript
    // device_controller.test.ts
    import { startDevice } from './device_controller';

    describe('startDevice', () => {
      it('should initialize device correctly', () => {
        // test logic here
      });
    });
    ```

## Commands
| Command         | Purpose                                              |
|-----------------|------------------------------------------------------|
| /add-module     | Scaffold and implement a new module                  |
| /write-tests    | Create and run tests for new or updated functionality|
| /import-export  | Share logic between modules using named exports      |
```
