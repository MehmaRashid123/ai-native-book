# Quickstart: Textbook Generation

This guide explains how to run the textbook generation process.

## Prerequisites

1.  **Docusaurus Project:** Ensure you have the Docusaurus project set up and all dependencies installed.
2.  **Agent Configuration:** The `agents.yaml` and `definitions.yaml` files in `.gemini/skills/` must be present and correctly configured.
3.  **Constitution:** The `.specify/memory/constitution.md` file must be up to date.

## Running the Generation

The textbook generation is driven by the tasks defined in the feature specification (`specs/001-textbook-generation/spec.md`).

To execute the plan, you would typically run a command like `/sp.implement` (not yet defined) which would:

1.  Read the `spec.md` to understand the tasks.
2.  Use the defined agents (`content_writer`, `lab_instructor`) to execute each task.
3.  The agents will use their skills and the prompts from the spec to generate the content.
4.  The generated content will be saved to the `output_file` path specified for each task (e.g., `docs/intro.mdx`).

## Example Workflow

1.  The system identifies the task `write_intro`.
2.  It selects the `content_writer` agent.
3.  The agent uses its `write_chapter` skill (or a similar skill).
4.  The prompt from the spec is used to generate the content for "Introduction to Physical AI".
5.  The output is saved to `docs/intro.mdx`.
6.  This process is repeated for all tasks.

## Validation

After generation, manually review the generated `.mdx` files in the `docs/` directory to ensure they are correct and meet the quality standards defined in the constitution.
