# ScenRelationInference

## Add Template Oject Models

Finished the object_label.json
Now I need to add two more types here: Flat_bottle, Fat_bottle, Plane.
(Bottle, no need to write it, just change it on existing files)
[Finished]
Template_type
0:Plane
1:Box
2:Cylinder
3:Flat_bottle (3 reliable plane Box) | In box system
4:Fat_bottle (1 reliable plane Box)

## Add Test Data Set

I need to generate a file for different data pose.  Actually, I don't need to generate, I can directly get it from csv file.

I have got the data, but how should I manage it? I want to seperate it 
currently.

Just parse pose from one file.

Read in csv file & leave the best render score. Then I can use it to make geometry Inference.

## Debug on System

Debug on current system  

### Adding system bug

Finished

### Synatx bugs on feature inference

TODO:
Add Visualization System!!!!(in Matlab or Python)
(The new visualization is in Matlab & we need to add a data log 
in geometry system)

Now I need to fix the problem of wrong projection. Once again, I make projection on the feature to detect the supporting status.

## Intergrate it into optimization