const { ipcRenderer } = require('electron');
var Plotly = require('plotly.js-dist');

var scannerCalibPlot = () => {
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
            },{
                x:[],
                y:[],
                z:[], 
                type: 'surface'
            }];
              
            Plotly.newPlot(plot, data, layout);

            ipcRenderer.on('data', (e,d)=> {
                console.log(d);

                d.forEach(el=>{
                    if(el.type=="plane") {
                        var X = []; 
                        var Y = [];
                        var Z = [];
                                                
                        let xmin=-100,xmax=100,ymin=-100,ymax=100;
                        let step=4;
if(el.n[2]!=0) {
    for(x=ymin; x<ymax; x+=step) {
        let zTemp = [];
        let yTemp = [];
        let xTemp = [];
        for (y=ymin; y<ymax; y+=step) {
            let z=-(el.n[3]+x*el.n[0]+y*el.n[1])/el.n[2] ;
            zTemp.push(z);
          yTemp.push(y);
          xTemp.push(x);
        }
        Z.push(zTemp);
        Y.push(yTemp);
        X.push(xTemp);
    }    
}

let layout_ = {
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

let data_ = [{
    x : points.x,
    y: points.y,
    z : points.z,
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
},{
    x:X,
    y:Y,
    z:Z, 
    type: 'surface'
}];

                        Plotly.newPlot(plot, data_,layout_);
                    }
                    else if(el.type=="point") {
                        let trace= {
                            x:[[el.xyz[0]]],
                            y:[[el.xyz[1]]],
                            z:[[el.xyz[2]]]
                        };
                        
                        points.x.push(el.xyz[0]);
                        points.y.push(el.xyz[1]);
                        points.z.push(el.xyz[2]);                        
                        Plotly.extendTraces(plot, trace, [0])
                    }
                });
            });                    
        }
    };
};


