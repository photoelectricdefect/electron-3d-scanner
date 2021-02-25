const { ipcRenderer } = require('electron');

var imageSlideshow = () => {
    let slidrContainer=document.getElementById("slidr-container");

    let slidr_= {};
    let images=[];
    let id=-1;

    return {
        init: ()=> {
            ipcRenderer.on('data', (e,data)=> {
                images=data.images;
                id=data.id;            
                let idx=images.findIndex(x=>x.id==id);
                
                slidr_ = slidr.create('slidr-container', {
                    breadcrumbs: true,
                });  

                let iter=0;
                
                for(let i=idx;iter<images.length;i++,iter++) {
                    if(i>=images.length) i=0;   

                    let img=document.createElement("img");    
                    img.dataset.slidr = i;
                    let blob = new Blob([images[i].data], {type: "image/jpeg"});
                    let frame = URL.createObjectURL(blob);
                    img.src = frame;
                    slidrContainer.appendChild(img);
                    slidr_.add('h',i);
                }

                slidr_.start();
            });        
        }
    };
};