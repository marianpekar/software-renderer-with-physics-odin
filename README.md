An experimental branch where Claude Code with *Sonnet 4.6* was used to generate verbose comments in every file.

The prompt:
```
Add very verbose comments to this codebase. In every file, comment every procedure, explaining what it does. If there are known algorithms or mathematical concepts, name them correctly. Inside procedure, explain each step verbosely.
```

The result is quite interesting, though I don't intend to merge this branch into the main branch.

Although the prompt sets focus on commenting procedures, algorithms, and mathematical concepts, AI also generated a summary at the top of each file and added comments to structs and type aliases. It also formatted the code, mostly by vertically aligning the left-hand and right-hand sides of expressions into columns.

At the time of execution, with the Claude Pro plan, the cost was 45% of the session limit (5 hours) and 7% of the weekly limit. The effort setting was *Medium*.