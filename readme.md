## Setup
- Prepare:
    You need to have ```gym``` and ```box2d-py``` installed. For the latter one, setup ```swig``` first. ```box2d-py``` can be installed seperately or by calling ```pip install gym[box2d]```
- Additional package (this one)
    Under the folder that contains ```gym-coil2d```
    ```
    pip install -e gym-coil2d
    ```

## Usage
- Initializing the environment by:
    ```python
    import gym

    gym.make('gym_coil2d:coil2d-v0')
    ```
- (s, a, r)
    - State
    - Action
    - Reward:
        - Each step
        - End reward