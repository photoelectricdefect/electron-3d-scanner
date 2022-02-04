const { ipcRenderer } = require('electron');
var Plotly = require('plotly.js-dist');

var scanPlot = () => {
    let plot=document.getElementById("plot");
    let points={
        x:[],
        y:[],
        z:[]
    };

    return {
        init: ()=> {
            var layout = {
                margin: {
                l: 0,
                r: 0,
                b: 0,
                t: 0
              },
              xaxis : {
                title : "x",
              },
              yaxis : {
                title : "y"
              },
              zaxis : {
                title : "z"
              }};     

            var data = [{
                x : [],
                y: [],
                z : [],
                mode: 'markers',
                marker: {
                  size: 6,
                  line: {
                    color: 'rgba(217, 217, 217, 0.14)',
                    width: 0.5
                  },
                  opacity: 0.8
                },
                type: 'scatter3d'
            }];
         
            Plotly.newPlot(plot, data, layout);

            ipcRenderer.on('data', (e,d)=> {                               
                if(d.type=="points") {
                    let trace= {
                        x:[d.data[0]],
                        y:[d.data[1]],
                        z:[d.data[2]]
                    };

                    Plotly.extendTraces(plot, trace, [0])
                }
                else if(d.type=="axis") {
                  console.log(d);
                  let len=50;

                  let axis={
                    x : [d.source[0],d.source[0]+len*d.direction[0]],
                    y: [d.source[1],d.source[1]+len*d.direction[1]],
                    z : [d.source[2],d.source[2]+len*d.direction[2]],
                    mode: "lines",
                    marker: {
                      size: 6,
                      line: {
                        color: 'rgba(217, 217, 217, 0.14)',
                        width: 0.5
                      },
                      opacity: 0.8
                    },    
                    line: {
                      color: "blue",
                      width: 5
                    },
                    type: "scatter3d"
                  };

                  let npoints=d.npoints;
                  let nedges=d.orbit_points[0].length/npoints;
                  let start=0;
                  Plotly.addTraces(plot,axis);
                  
                  console.log(npoints);
                  console.log(nedges);

                  for(let i=0;i<nedges;i++) {
                    //console.log(d.orbit_points[0].slice(start,npoints));
                    // Returns a random number:
                    var randomColor = Math.floor(Math.random()*16777215).toString(16);
                    var tmpx=[];
                    var tmpy=[];
                    var tmpz=[];

                    for(var j=0;j<npoints;j++) {
                      tmpx.push(d.orbit_points[0][start+j]);
                      tmpy.push(d.orbit_points[1][start+j]);
                      tmpz.push(d.orbit_points[2][start+j]);
                    }

                    let orbitPoints={
                      x : tmpx,
                      y: tmpy,
                      z : tmpz,
                      mode: "markers",
                      marker: {
                        size: 6,
                        line: {
                          color: "#"+randomColor,
                          width: 0.5
                        },
                        opacity: 0.8
                      },    
                      type: "scatter3d"
                    };
  
                    let centerPoints={
                      x : [d.center_points[0][i]],
                      y: [d.center_points[1][i]],
                      z : [d.center_points[2][i]],
                      mode: "markers",
                      marker: {
                        size: 6,
                        line: {
                          color: "#"+randomColor,
                          width: 0.5
                        },
                        opacity: 0.8
                      },    
                      type: "scatter3d"
                    };
  
                    start+=npoints;
                    Plotly.addTraces(plot,orbitPoints);
                    Plotly.addTraces(plot,centerPoints);                    
                  }
                  

                  // let trace= {
                  //     x:[d.data[0]],
                  //     y:[d.data[1]],
                  //     z:[d.data[2]]
                  // };

                  // Plotly.extendTraces(plot, trace, [0])
              }                              
            });                    
        }
    };
};


