# -*- coding: utf-8 -*-
"""Flugramapir.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/drive/150P9OmFnG3EFtijQ3-MzYiKxPn2IevCq
"""

!apt-get install -y graphviz
!pip install graphviz
!pip install pydot

from graphviz import Digraph

# Criação do gráfico de fluxograma com Graphviz
dot = Digraph(comment='Fluxograma do Sistema ESP32 com Sensor PIR e LDR')

# Adicionando nós para o fluxograma
dot.node('A', 'Início', shape='rectangle')
dot.node('B', 'Presença detectada?', shape='diamond')
dot.node('C', 'LDR  >=30 e < 10?', shape='diamond')
dot.node('D', 'Enviar dados para coordenador', shape='rectangle')
dot.node('E', 'Coordenador envia dados para outro end device', shape='rectangle')
dot.node('F', 'Ligar LED', shape='rectangle')
dot.node('G', 'LDR  >=30 e < 10', shape='diamond')
dot.node('H', "Mensagem: 'Presença detectada, iluminação suficiente'", shape='rectangle')
dot.node('I', 'Fim', shape='rectangle')

# Adicionando as arestas (relações entre os nós)
dot.edge('A', 'B')
dot.edge('B', 'C', label='Sim')
dot.edge('B', 'G', label='Não')
dot.edge('C', 'D', label='Sim')
dot.edge('C', 'I', label='Não')
dot.edge('D', 'E')
dot.edge('E', 'F')
dot.edge('G', 'H')
dot.edge('H', 'I')

# Renderizando o fluxograma
dot.render('fluxograma_esp32', format='png', cleanup=True)

# Exibindo o fluxograma gerado
from IPython.display import Image
Image('fluxograma_esp32.png')