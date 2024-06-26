> [!summary] Definition
> Central Pattern Generators (CPG) are Neural Networks that produce rhythmic outputs without external sensory inputs. They can be used to produce rhythmic signals for motor control.

In living organisms, they are responsible for rhythmic motor behavior such as:
- walking
- chewing
- breathing 
- swimming

To be classified as a rhythmic generator, a CPG requires:
1. two or more processes that interact such that each process sequentially increases and decreases, and
2. that, as a result of this interaction, the system repeatedly returns to its starting condition.

## Mechanisms of rhythm generation
Rhythm generation in CPG networks depends on the intrinsic properties of CPG neurons and their synaptic connections. There are two general mechanisms for rhythm generation: <mark style="background: #ADCCFFA6;">pacemaker/follower</mark> and <mark style="background: #D2B3FFA6;">reciprocal inhibition</mark>.
- In a network driven by a [[Central Pattern Generator#Pacemaker Model|pacemaker]], <mark style="background: #ADCCFFA6;">one or more neurons</mark> act as a core oscillator (pacemaker) that <mark style="background: #ADCCFFA6;">drives other</mark>, non-bursting neurons (follower) <mark style="background: #ADCCFFA6;">into a rhythmic pattern</mark>. 
- In a network driven by reciprocal inhibition, <mark style="background: #D2B3FFA6;">two</mark> (groups of) <mark style="background: #D2B3FFA6;">neurons reciprocally inhibit each other</mark>. Such networks are known as [[Central Pattern Generator#Half-Center Model|half-center oscillators]]. The neurons are not rhythmically active when isolated, but they can produce alternating patterns of activity when coupled by inhibitory connections. 

## Half-Center Model
Two neurons, the Flexor (F) and Extendor (E), rhythmically inhibit each other. A Diver neuron (D) constantly stimulates inter-neurons (solid white) through exciting connections. Through inhibiting connections, they inhibit each other rhythmically.
![[Pasted image 20240301121802.png#invert|200]]
- It can be modeled by the [[Matsuoka Oscillator]]

## Pacemaker Model
Uses the intrinsic rhythmic characteristic of a pacemaker neuron (S) to create oscillations.
![[Pasted image 20240301122211.png#invert|200]]
