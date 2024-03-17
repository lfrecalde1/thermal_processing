#!/usr/bin/env python
import scipy.linalg
import numpy as np
import time
import matplotlib.pyplot as plt
import scipy.io
import rospy
from sklearn.cluster import KMeans
from matplotlib.colors import ListedColormap
import random
from sklearn.metrics import silhouette_score
from matplotlib.ticker import MaxNLocator

# Load Data
Data = scipy.io.loadmat('Data_1.mat') 
Data_points = Data['Data_matrix'] 

# Create New matrix with an appropiate dimension
X = Data_points.T

# Check for the number of clusters
silhouette_scores = []
for n_cluster in range(2, 20):
    kmeans = KMeans(n_clusters=n_cluster).fit(X)
    label = kmeans.labels_
    sil_coeff = silhouette_score(X, label, metric='euclidean')
    silhouette_scores.append(sil_coeff)

# Compute the value of cluster
max_value = max(silhouette_scores)
k = silhouette_scores.index(max_value) + 2

# Compute centroids and clusters
# Create KMeans instance
kmeans = KMeans(n_clusters=k, random_state=0)
# Fit the model
kmeans.fit(X)
# Cluster centroids
centroids = kmeans.cluster_centers_
# Predict the cluster for each sample
labels = kmeans.predict(X)
# Generate a list of random colors
colors = ListedColormap([plt.cm.Paired(random.random()) for _ in range(k)])


# Reshape matrix
aux = centroids.T

# Create values with the location of the arrays and an empty matrix
arrays_values = np.array([0, -11, -23])
aux_centroids = np.zeros((aux.shape[0], aux.shape[1]))

# Compute new values 
for i in range(0, arrays_values.shape[0]):
    for j in range(0, aux.shape[1]):
        distance = np.linalg.norm(arrays_values[i] - aux[1, j])
        if distance < 1.0:
            aux_centroids[0, j] = i + 1
            aux_centroids[1, j] = aux[0, j]

# Plot new results with the number of each array and the distance to the each hot spot
x_min, x_max = np.min(aux_centroids[0, :]) -1, np.max(aux_centroids[0,:]) +1
y_min, y_max = 0, np.max(aux_centroids[1,:]) +5
# Plotting the clusters and centroids with random colors
plt.grid(color='#949494', linestyle='-.', linewidth=0.5)
for i in range(0, aux_centroids.shape[1]):
    # Plot data points belonging to cluster i
    #plt.scatter(X[labels == i, 0], X[labels == i, 1], s=100, color=colors(i))
    
    # Plot the centroid of cluster i
    plt.scatter(aux_centroids[0, i], aux_centroids[1, i], s=100, color=colors(i), marker='o', edgecolor='black')
    plt.text(aux_centroids[0, i], aux_centroids[1, i]+1.5, f'({aux_centroids[0, i]}, {round(aux_centroids[1, i],2)})', color='black', ha='center', va='bottom')

plt.title('Distribucion de Puntos Calientes Por Simulacion')
plt.xlabel('Arreglo fotovoltaico')
plt.ylabel('Distancia [m]')
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)
plt.gca().xaxis.set_major_locator(MaxNLocator(integer=True))
# Show the plot
plt.savefig('Puntos_Calientes_Simulacion_test.pdf', format='pdf')
plt.show()