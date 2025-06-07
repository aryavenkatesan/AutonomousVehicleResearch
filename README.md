# AutonomousVehicleResearch

Here are the commands to setup f1tenthgym: 
python --version | 
cd desktop/hw/f1testing  | 
brew install pyenv  | 
brew install pyenv-virtualenv  | 
pyenv install 3.9.18 | 
pyenv virtualenv 3.9.18 gym_env | 
eval "$(pyenv init -)" | 
eval "$(pyenv virtualenv-init -)" | 
source ~/.zshrc | 
python --version | 
pyenv virtualenvs | 
cd f1tenth_gym | 
cd examples | 
cd .. | 
pip install --upgrade pip 

_____________

mkdir f1new | 
cd f1new | 
pyenv local 3.9.18 | 
python --version | 
pip install --upgrade pip | 
pyenv virtualenv 3.9.18 gym_env | 
pyenv activate gym_env | 
pip install git+https://github.com/f1tenth/f1tenth_gym.git | 
git clone https://github.com/f1tenth/f1tenth_gym.git | 
cd f1tenth_gym | 
pip install -e . | 
pip3 install pyglet==1.5.20 | 
pip install -e . | 
cd examples | 
pip3 install pyglet==1.5.20 | 
python3 waypoint_follow.py 

____________

Fast env setup:

eval "$(pyenv init -)" && eval "$(pyenv virtualenv-init -)" && source ~/.zshrc && pyenv virtualenv 3.9.18 gym_env ; pyenv activate gym_env