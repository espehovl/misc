# -*- coding: utf-8 -*-
"""
Created on Fri Mar 27 18:34:50 2020

@author: Espen
"""
import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#Return the distance between two 2D coordinates
def distance(t1,t2):
	dx = abs(t1[0]-t2[0])
	dy = abs(t1[1]-t2[1])
	return np.sqrt(dx**2 + dy**2)


class Particle:
	#Initialize, xy is a tuple
	def __init__(self,xy, ID = 0):
		self.x = xy[0]
		self.y = xy[1]
		self.status = "good"
		self.color = "limegreen" 
		self.sickTime = 0
		self.id = ID
		self.immune = False
		self.infects = 0 #How many other has this particle infected?
	
	#Move particle one step in desired direction (allows for multiple movements)
	def stepMove(self, direction):
		for c in direction:
			if   c == "u": self.y -= 1
			elif c == "d": self.y += 1
			elif c == "l": self.x -= 1
			elif c == "r": self.x += 1
			else: None
	
	#Place particle on grid
	def place(self, newX, newY):
		self.x = newX
		self.y = newY
	
	#Infect particle
	def infect(self, prob = 1):
		reciprocal = int(round(1/prob,0))
		selection = [False for i in range(reciprocal-1)]
		selection.append(True)
		
		if random.choice(selection):
			self.status = "infected"
			self.color = "tomato" #Red, for infected

	#Heal particle
	def heal(self):
		self.status = "healed"
		self.color = "darkgrey"
		self.immune = True
	
	#Get coord tuple
	def getXY(self):
		return (self.x, self.y)
	
	#Print coordinate of particle
	def printXY(self):
		print("("+str(self.x)+","+str(self.y)+")")

##########################################################################################

class Simulate:
	#Initialize
	def __init__(self, noParticles, gridsize,  initialInfected, 
			  stationaryParticles = 0, radius = 1, probability = 1):
		self.noParticles = noParticles
		self.dim = gridsize
		self.counter = 0
		self.steps = self.counter + 1
		self.healTime = 30
		self.infectRadius = radius
		self.infectProbability = probability
		self.initialInfected = initialInfected
		self.noStationaryParticles = int(stationaryParticles * self.noParticles / 100)
		#Some variables for statistics
		self.totalHealed = 0
		self.healed = [0]
		self.totalSick = self.initialInfected
		self.currentlySick = self.initialInfected
		self.sick = [self.initialInfected]
		self.healthy = [self.noParticles-self.initialInfected]
		#Create the grid
		self.grid = np.array([[0 for j in range(self.dim)] for i in range(self.dim)])
		#Add unique Particle objects to particles
		self.particles = []
		self.occupied = [] 	#List of occupied coordinate tuples
		for i in range(self.noParticles):
			x = random.randint(0,self.dim-1)
			y = random.randint(0,self.dim-1)
			while (x,y) in self.occupied:
				#Find new coords if they are already occupied
				x = random.randint(0,self.dim-1)
				y = random.randint(0,self.dim-1)
			self.particles.append(Particle((x,y),i+1))
			self.occupied.append((x,y))
			self.grid[y][x] = 1
		
		#Initial infected
		for i in range(self.initialInfected):
			self.particles[i].infect()
			
		self.plot()
	
	#Update the grid
	def updateGrid(self):
		self.grid.fill(0)
		self.occupied = [] #Clear, ready for refresh
		for p in self.particles:
			xy = p.getXY()
			x = xy[0]
			y = xy[1]
			self.grid[y][x] = 1
			self.occupied.append(xy)
	
	#Print grid
	def printGrid(self):
		print(self.grid)
	
	#Perform multiple steps, either n iterations or until no more infections
	def performSteps(self, n = -1):
		print(f"Performing iterations...")
		if n == -1:
			print("Auto-iterations")
			ctr = 0
			while self.currentlySick != 0:
				ctr+=1
				self.perform()
				print(ctr,end="  ")

		elif n > 0:
			print(f"{n} steps")
			for i in range(n):
				self.perform()
			print()
		else: 
			print("Not a valid number of steps.")
			return
		
		#Make a "stationary" image at the end of the GIF
		for i in range(10):
			self.plot(True, i)
		print("Tail added")
			
		print(f"Simulation performed with {self.counter} steps.")

	
	#Perform one iteration
	def perform(self):
		index = 0
		for p in self.particles:
			if p.status == "infected":
				p.sickTime += 1
				if p.sickTime >= self.healTime:
					p.heal()
					self.currentlySick-=1
					self.totalHealed += 1
				
			xy = p.getXY()
			x, y = xy[0], xy[1]
			
			#The possible movements for particles, will be updated below
			options = ["u","d","l","r", "n"] 
			
			#Locate other particle coordinates
			otherOccupied = [c for c in self.occupied]
			otherOccupied.remove(xy) 

			#Remove illegal movements from options
			if x == 0 or (x-1,y) in otherOccupied: 			options.remove("l")
			if x == self.dim-1 or (x+1,y) in otherOccupied: options.remove("r")
			if y == 0 or (x,y-1) in otherOccupied: 			options.remove("u")
			if y == self.dim-1 or (x,y+1) in otherOccupied: options.remove("d")
			
			if len(self.particles)-1-index > self.noStationaryParticles:# and p.status!= "infected": #Don't move when infected!
				p.stepMove(random.choice(options))
			self.updateGrid()
			index += 1
		
		self.counter += 1 #Keep track of amount of iterations
		self.steps = self.counter
		
		#Infect neighbours
		for p in self.particles:
			neighbours = self.getNeighbours(p)
			for n in neighbours:
				if n.status == "infected" and p.immune == False and p.status != "infected":
					p.infect(self.infectProbability)
					if p.status == "infected":
						n.infects += 1
						self.currentlySick += 1
						self.totalSick += 1
		
		self.healthy.append(self.noParticles - self.currentlySick)
		self.sick.append(self.currentlySick)
		self.healed.append(self.totalHealed)
					
		#Graphical representation parts vvv
		self.plot()

		
	#Get neighbour particles
	def getNeighbours(self, particle):
		neighbours = []
		for p in self.particles:
			if p.id != particle.id and distance(p.getXY(),particle.getXY()) <= self.infectRadius:
				neighbours.append(p)
		return neighbours		
	
	#Plot data
	def plot(self, tail = False, extraIndex = 0):
		#Plot and save plot as .png
		plt.ioff()
		fig,(ax1,ax2) = plt.subplots(1,2,figsize=(10,5))


		ax2.set_xlim([0,self.steps])
		ax2.set_ylim([0,self.noParticles])
		ax2.set_ylabel("# people")
		
		R_raw = 0		 
		for p in self.particles:
			ax1.scatter(p.getXY()[0],p.getXY()[1], s = 3000/self.dim, color = p.color)
			if p.status == "infected" and p.sickTime > 0:
				R_raw += p.infects / p.sickTime * self.healTime
		ax2.fill_between(range(self.counter+1),self.noParticles,[self.noParticles-self.healed[i] for i in range(len(self.healed))],color="darkgrey", alpha = 0.5, label = "Immune")
		ax2.fill_between(range(self.counter+1),self.sick, color = "tomato", alpha = 0.5, label = "Infected")
		ax2.fill_between(range(self.counter+1),[self.noParticles-self.healed[i] for i in range(len(self.healed))],self.sick, color = "limegreen", alpha = 0.5, label = "Healthy")
		
		ax2.text((self.steps*1.02),int(self.noParticles*0.8),"Stats:")
		ax2.text((self.steps*1.02),int(self.noParticles*0.75),f"Total infected: {self.totalSick}", fontsize = 8)
		ax2.text((self.steps*1.02),int(self.noParticles*0.7),f"Curr. infected: {self.currentlySick}",fontsize = 8)
		ax2.text((self.steps*1.02),int(self.noParticles*0.65),f"Healthy: {self.healthy[-1]}",fontsize = 8)
		ax2.text((self.steps*1.02),int(self.noParticles*0.6),f"Immune: {self.healed[-1]}",fontsize = 8)
		ax2.text((self.steps*1.02),int(self.noParticles*0.25),f"Population: {self.noParticles}", fontsize = 8)
		ax2.text((self.steps*1.02),int(self.noParticles*0.2), f"In quarantine:",fontsize = 8)
		ax2.text((self.steps*1.02),int(self.noParticles*0.15), f"{int(self.noStationaryParticles / self.noParticles * 100)} %",fontsize = 12)
		try:
			ax2.text(self.steps*1.02,int (self.noParticles*0.10),r"$R=$"+f"{np.round(R_raw/self.currentlySick,2)}")
		except:
			ax2.text(self.steps*1.02,int (self.noParticles*0.10),r"$R=$"+f"NaN")
		ax2.text((self.steps*1.02), int(self.noParticles*0.03),f"Time: {self.counter}",fontsize = 12)
		
		ax2.legend(loc='upper left', bbox_to_anchor=(1, 1))
		ax2.set_xticklabels([])
		ax2.set_yticklabels([])
		ax2.set_xticks([])
		ax2.set_yticks([])
		ax2.grid(False)
#		ax1.grid(True, color = "k", alpha = 0.15)
#		ax1.set_xticks([i + 1/2 for i in range(0,self.dim)])
#		ax1.set_yticks([i + 1/2 for i in range(0,self.dim)])
		ax1.set_xticklabels([])
		ax1.set_yticklabels([])
		ax1.set_xticks([])
		ax1.set_yticks([])
		ax1.set_xlim([-0.4,self.dim-0.5])
		ax1.set_ylim([-0.4,self.dim-0.5])
		
		#plt.show()
		plt.tight_layout()
		if not tail:
			fig.savefig(f"pictures/corona{'{:04d}'.format(self.counter)}.png")
		elif tail:
			fig.savefig(f"pictures/corona{'{:04d}'.format(self.counter + extraIndex + 1)}.png")
		plt.clf()
	
	#Save a GIF
	def saveGIF(self, filename = "corona"):
		from PIL import Image
		import glob
	
		print("Creating GIF...")
		# Create the frames
		frames = []
		imgs = glob.glob("pictures/*.png")
		for i in imgs:
			new_frame = Image.open(i)
			frames.append(new_frame)
			# Save into a GIF file that loops 	forever
			frames[0].save(filename+'.gif', format='GIF', append_images=frames[1:], save_all=True, duration=300, loop=0)

		print("GIF conversion complete!")