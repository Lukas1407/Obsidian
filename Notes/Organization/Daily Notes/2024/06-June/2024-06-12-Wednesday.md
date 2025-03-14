---
created: 2024-06-12 07:19
exercise: false 
reading: 0
---
tags:: [[+Daily Notes]]

# Wednesday, June 12, 2024

<< [[Organization/Daily Notes/2024/06-June/2024-06-11-Tuesday|Yesterday]] | [[Organization/Daily Notes/2024/06-June/2024-06-13-Thursday|Tomorrow]] >>

---
### 📅 Daily Questions
##### 🌜 Last night, after work, I...
- Exercise YT Japanese

##### 🙌 One thing I'm excited about right now is...
- Just doing lectures

##### 🚀 Things I plan to accomplish today is...
- [ ] Write daily notes
- [ ] 7:20-11:00 Formale Systeme
- [ ] 11:30-17:00 Formale Systeme

##### 👎 One thing I'm struggling with today is...
- 
---
### Space for some Thoughts

--- 
### Habits
```dataviewjs
dv.span("**🏋️ Exercise 🏋️**") /* optional ⏹️💤⚡⚠🧩↑↓⏳📔💾📁📝🔄📝🔀⌨️🕸️📅🔍✨ */
const calendarData = {
    colors: {    // (optional) defaults to green
        blue:        ["#8cb9ff", "#69a3ff", "#428bff", "#1872ff", "#0058e2"], // first entry is considered default if supplied
        green:       ["#c6e48b", "#7bc96f", "#49af5d", "#2e8840", "#196127"],
        red:         ["#ff9e82", "#ff7b55", "#ff4d1a", "#e73400", "#bd2a00"],
        orange:      ["#ffa244", "#fd7f00", "#dd6f00", "#bf6000", "#9b4e00"],
        pink:        ["#ff96cb", "#ff70b8", "#ff3a9d", "#ee0077", "#c30062"],
        orangeToRed: ["#ffdf04", "#ffbe04", "#ff9a03", "#ff6d02", "#ff2c01"]
    },
    showCurrentDayBorder: true, // (optional) defaults to true
    defaultEntryIntensity: 4,   // (optional) defaults to 4
    intensityScaleStart: 10,    // (optional) defaults to lowest value passed to entries.intensity
    intensityScaleEnd: 100,     // (optional) defaults to highest value passed to entries.intensity
    entries: [],                // (required) populated in the DataviewJS loop below
}

//DataviewJS loop
for (let page of dv.pages('"Organization/Daily Notes"').where(p=>p.exercise)) {
    // Extract the date parts from the file name
	const parts = page.file.name.split("-");
	const year = parts[0]; // yyyy
	const month = parts[1]; // mm
	const day = parts[2]; // dd
	
	// Construct the date string in the required format
	const dateString = `${year}-${month}-${day}`;
	
	// Now, use dateString for the date property
	calendarData.entries.push({
	    date: dateString,
	    intensity: page.exercise,
	    color: "green",   
	});
}

renderHeatmapCalendar(this.container, calendarData);
```
```dataviewjs
dv.span("**📔 Reading 📔**") /* optional ⏹️💤⚡⚠🧩↑↓⏳📔💾📁📝🔄📝🔀⌨️🕸️📅🔍✨ */
const calendarData = {
    intensityScaleEnd: 30,
    colors: {
        red: ["#ff9e82","#ff7b55","#ff4d1a","#e73400","#bd2a00",
        "hsl(132, 90%, 40%)"] //last one green
    },
    entries: []
};

// DataviewJS loop
for (let page of dv.pages('"Organization/Daily Notes"').where(p => p.reading)) {
    // Extract the date parts from the file name
    const parts = page.file.name.split("-");
    const year = parts[0]; // yyyy
    const month = parts[1]; // mm
    const day = parts[2]; // dd
    const dateString = `${year}-${month}-${day}`;

    calendarData.entries.push({
        date: dateString,
        intensity: page.reading
    })
}

renderHeatmapCalendar(this.container, calendarData);
```
--- 
### Notes created today
```dataview
List FROM "" WHERE file.cday = date("2024-06-12") SORT file.ctime asc
```
