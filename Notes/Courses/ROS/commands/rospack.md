allows to get information about the package
## Find
`rospack find [package_name]` 
Example: 
		`rospack find roscpp` returns `/opt/ros/noetic/share/roscpp`

## Dependencies
`rospack depends1 <package_name>`
Example:
	`rospack depends1 beginner_tutorials` returns the <mark style="background: #FFB86CA6;">first-order dependencies</mark>
	