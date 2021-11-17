## Setup

- You need to have `gym` and `box2d-py` installed. 
    1. Setup [Swig](http://www.swig.org/download.html) first if you don't have it installed.
    2. install Pyglet by:
            `pip install pyglet`
    3. `box2d-py` can be installed seperately or by calling
        `pip install gym[box2d]`
- Additional package (this one) Under the folder that contains `gym_coil2d`
    - `pip install -e gym_coil2d`

## Usage
- Run `main_spiral.py`
- Start with pressing "s" key
- Exit with pressing "ESC" key
- To use the simulation environment:
    `env = gym.make('gym_coil2d:coil2d-v0')`

## Note
- "bezier.py" is a previous code that has not been updated.

## Ref
- [How to create new environments for Gym](https://github.com/openai/gym/blob/master/docs/creating-environments.md)