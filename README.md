# 15PuzzlesSolver

A Python-based solver for the classic 15 Puzzle using the Manhattan distance heuristic.

## Project Overview

The 15 Puzzle is a sliding puzzle consisting of a 4x4 grid of numbered tiles with one empty space. The objective is to rearrange the tiles from a given scrambled state to the solved configuration by sliding tiles into the empty space. This project implements an efficient solver that uses the A* search algorithm with the Manhattan distance heuristic to find an optimal sequence of moves to solve any valid puzzle configuration.

This solver is designed to demonstrate algorithmic problem-solving, heuristic search techniques, and clean Python implementation. It serves as a portfolio project showcasing proficiency in artificial intelligence concepts, data structures, and software engineering best practices.

## Core Features

- A* search algorithm implementation for optimal puzzle solving
- Manhattan distance heuristic for efficient state evaluation
- Command-line interface for flexible puzzle input and configuration
- Support for custom puzzle strings representing move sequences
- Configurable search depth and timeout parameters
- Clean, modular code structure for maintainability and extensibility

## Technology Stack

- **Python**: Chosen for its readability, extensive standard library, and strong support for algorithmic implementations. Python's built-in data structures (lists, dictionaries, sets) are well-suited for representing puzzle states and tracking visited configurations. The language's dynamic typing and rapid prototyping capabilities enable efficient development and testing of search algorithms.

## Architecture and Design Decisions

The codebase is organized into three primary modules, each with a distinct responsibility:

- **MySolver.py**: Contains the core solving logic, including the A* search algorithm and the Manhattan distance heuristic calculation. This module is designed to be independent of input/output concerns, allowing it to be reused in different contexts.

- **Run.py**: Serves as the entry point for command-line execution. It parses user input, validates parameters, and orchestrates the solving process by calling the solver module.

- **Puzzle representation**: Puzzle states are represented as tuples of integers, providing immutability and hashability for efficient use as dictionary keys in the visited set. This design choice prevents accidental state mutation and enables O(1) lookup for visited states.

The separation of concerns between the solver logic and the user interface allows for easy testing and future extension. The solver can be integrated into web applications, GUI programs, or other interfaces without modification.

## Installation and Setup

1. Ensure Python 3.6 or later is installed on your system.

2. Clone the repository:
   ```
   git clone https://github.com/tvay11/15PuzzlesSolver.git
   cd 15PuzzlesSolver
   ```

3. Run the solver with a puzzle string:
   ```
   python Run.py 3 120 -m PUZZLE
   ```
   Replace `PUZZLE` with a string of moves such as `ULDLUURDLDRRULLDRULURRDD` or `UULDLUULDRRURDDLDLULURULDDDRUURULDRDRUULDRDDL`.

   The parameters are:
   - `3`: Number of random shuffles (used for generating test puzzles)
   - `120`: Timeout in seconds for the solver
   - `-m`: Flag indicating a manual puzzle string follows
   - `PUZZLE`: The puzzle string representing the initial configuration

## Future Scope and Key Learnings

This project provided hands-on experience with heuristic search algorithms, state space representation, and performance optimization in Python. Key learnings include the importance of heuristic selection in A* search, efficient state representation for memory-constrained environments, and the trade-offs between optimality and computational cost.

Potential future enhancements include:

- Implementing additional heuristics (e.g., linear conflict, pattern databases) for improved performance on harder puzzles
- Adding a graphical user interface for interactive puzzle solving
- Supporting larger puzzle sizes (e.g., 24 Puzzle on a 5x5 grid)
- Parallelizing the search algorithm to leverage multi-core processors
- Providing a web-based interface using Flask or FastAPI for remote access