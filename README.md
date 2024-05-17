# prova1-m6

## Instalação

1. Clone o repositório em sua máquina:

```bash
git clone https://github.com/ItsVasconcellos/prova1-m6
```


2. Abra o repositório em um terminal e entre na pasta meu_workspace:
```bash
cd meu_workspace
```

3. Após isso, execute o comando a seguir para executar a build do pacote ros:
```bash 
colcon build
```

4. Crie o local_setup para seu terminal
```bash
source install/local_setup.bash
```

5. Rode o comando a baixo:
```bash 
ros2 run turtlesim turtlesim_node 
```

6. Rode o comando para executar o comando:
```bash 
python3 cli.py --vx 0.0 --vy 1.0 --vt 0.0 -t 1000 
```