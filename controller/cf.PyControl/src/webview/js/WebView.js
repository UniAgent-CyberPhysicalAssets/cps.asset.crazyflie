class WebView {
    constructor() {
        this.picsUrls = new Array();
        this.uavNames = new Array();
        this.divArray = new Array();
        this.canvasArray = new Array();
        this.removedPics = new Array();
        this.currendPics = new Array();
        this.checkComplet = false;
        this.updateComplet = false;
        this.findNewPicture = false;
        this.checkCount = 100;
        this.onUdate = false;
        this.canvasHeight = 710;
        this.canvasWidth = 1900;
        this.divWidth = 122;
        this.colorSaved = false;
        this.backgroundColorWhite = null;
        this.borderColorBlack = null;
        this.backgroundColorGrey = "#B0B0B0";
        this.borderColorRed = "red";
        this.createStartStopCallback();
    }

    createStartStopCallback() {
        const constThis = this;
        this.butStartStop = document.getElementById("button_start_stop");
        this.butStartStop.onclick = async function () {
            if (constThis.onUdate) {
                constThis.butStartStop.innerHTML = "Start Update";
                constThis.onUdate = false;
                return;
            }
            constThis.onUdate = true;
            constThis.butStartStop.innerHTML = "Stop Update";
            constThis.clearData(true);
            await new Promise(resolve => setTimeout(resolve, 300));
            while(constThis.onUdate) {
                constThis.updateData();
                while (!constThis.updateComplet) {
                    await new Promise(resolve => setTimeout(resolve, 20));
                }
                constThis.updateComplet = false;
            }
        }
    }

    clearData(alsoUrls) {
        if(alsoUrls) {
            var element;
            for (let i = 0; i < this.divArray.length; i++) {
                element = document.getElementById(this.divArray.at(i));
                if(element != null) {
                    console.log("ok div", this.divArray.at(i));
                    element.remove();
                }
            }
            for (let i = 0; i < this.canvasArray.length; i++) {
                element = document.getElementById(this.canvasArray.at(i));
                if(element != null) {
                    console.log("ok canvas", this.canvasArray.at(i));
                    element.remove();
                }
            }
            for (let i = 0; i < this.uavNames.length; i++) {
                element = document.getElementById(this.uavNames.at(i));
                if(element != null) {
                    console.log("ok pic", this.uavNames.at(i));
                    element.remove();
                }
            }
            this.divArray = null;
            this.divArray = new Array();
            this.canvasArray = null;
            this.canvasArray = new Array();
            this.uavNames = null;
            this.uavNames = new Array();
            this.picsUrls = null;
            this.picsUrls = new Array();
            this.removedPics = null;
            this.removedPics = new Array();    
            return;
        }

        var findRemovedDiv = false;
        var findRemovedCanvas = false;
        var findRemovedPic = false;
        var countRemoved = 0;
        for (let i = 0; i < this.divArray.length; i++) {
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i)) == 0) {
                    findRemovedPic = true;
                }
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i) + "_div") == 0) {
                    findRemovedDiv = true;
                }
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i) + "_canvas") == 0) {
                    findRemovedCanvas = true;
                }
            }
            if(findRemovedDiv) {
                findRemovedDiv = false;
                countRemoved++;
            } else {
                document.getElementById(this.divArray.at(i)).remove();
            }
            if(findRemovedCanvas) {
                findRemovedCanvas = false;
                countRemoved++;
            } else {
                document.getElementById(this.canvasArray.at(i)).remove();
            }
            if(findRemovedPic) {
                findRemovedPic = false;
                countRemoved++;
            } else {
                document.getElementById(this.uavNames.at(i)).remove();
            }
        }
        var findRemoved;
        for(let i = 0; i < this.divArray.length; i++) {
            findRemoved = false;
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.divArray.at(i).localeCompare(this.removedPics.at(i2) + "_div") == 0) {
                    findRemoved = true;
                }
            } 
            if(!findRemoved) {
                this.divArray.splice(i, 1);
            }
        }
        for(let i = 0; i < this.canvasArray.length; i++) {
            findRemoved = false;
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.canvasArray.at(i).localeCompare(this.removedPics.at(i2) + "_canvas") == 0) {
                    findRemoved = true;
                }
            } 
            if(!findRemoved) {
                this.canvasArray.splice(i, 1);
            }
        }
    }

    async updateData() {
        this.getPicsUrlsAndNames();
        while (!this.checkComplet) {
            await new Promise(resolve => setTimeout(resolve, 20));
        }
        this.checkComplet = false;
        if(this.findNewPicture) {
            this.outputTextInfo((this.picsUrls.length + " SM's finding: "), true);
            this.clearData(false);
            for (let i = 0; i < this.picsUrls.length; i++) {
                this.outputTextInfo(this.picsUrls.at(i), false);
                this.loadSMPictuere(this.picsUrls.at(i), this.uavNames.at(i), (80 + (i * this.canvasHeight) + (i * 5)));
            }
            this.findNewPicture = false;
        } else {
            this.updateSMPictuere();
        }
        await new Promise(resolve => setTimeout(resolve, 300));

        this.updateComplet = true;
    }

    updateSMPictuere() {
        var imgs = new Array(this.uavNames.length);
        var div = new Array(this.uavNames.length);
        var canvas = new Array(this.uavNames.length);
        var findRemoved;
        for(let i = 0; i < this.uavNames.length; i++) {
            imgs[i] = document.getElementById(this.uavNames.at(i));
            imgs[i].src = this.picsUrls.at(i) + "?" + Date.now().toString();
            imgs[i].onload();

            div[i] = document.getElementById(this.uavNames.at(i) + "_div");
            canvas[i] = document.getElementById(this.uavNames.at(i) + "_canvas");
            findRemoved = false;
            for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                if(this.removedPics.at(i2).localeCompare(this.uavNames.at(i)) == 0) {
                    findRemoved = true;
                }
            }
            if(findRemoved) {
                div[i].style.backgroundColor = this.backgroundColorGrey;
                div[i].style.borderColor = this.borderColorRed;
                canvas[i].style.borderColor = this.borderColorRed;
            } else {
                div[i].style.backgroundColor = this.backgroundColorWhite;
                div[i].style.borderColor = this.borderColorBlack;
                canvas[i].style.borderColor = this.borderColorBlack;
            }
        }
    }

    loadSMPictuere(imagePath, uavName, topPose) {
        var div = document.createElement('div');
        div.id = uavName + "_div";
        div.width = this.divWidth;
        div.height = this.canvasHeight;
        div.style.position = "absolute";
        div.style.top = topPose.toString() + "px";
        div.style.left = "10px"
        div.style.border = "1px solid";
        div.style.padding = "25px";
        div.style.fontSize = "25px";
        div.innerHTML = uavName;
        this.divArray.push(div.id);

        var canvas = document.createElement('canvas');
        canvas.id = uavName + "_canvas";
        canvas.width = this.canvasWidth;
        canvas.height = this.canvasHeight;
        canvas.style.position = "absolute";
        canvas.style.top = topPose.toString() + "px";
        canvas.style.left = (this.divWidth + 10).toString() + "px";
        canvas.style.border = "1px solid";
        this.canvasArray.push(canvas.id);
        
        var body = document.getElementsByTagName("body")[0];
        body.appendChild(canvas);
        body.appendChild(div);

        var ctx = canvas.getContext("2d");
        var img = new Image();
        img.src = imagePath + "?" + Date.now().toString();
        img.onload = function () {
            // ctx.drawImage(img, 0, 0);
            // Set the desired width or height for scaling
            var desiredWidth = 1900; // for example, you can set this to any value you want
            var desiredHeight;
            // Calculate the aspect ratio
            var aspectRatio = img.width / img.height;

            // Calculate the new height to maintain the aspect ratio
            desiredHeight = desiredWidth / aspectRatio;

            // Clear the canvas before drawing
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // Draw the image with the new dimensions
            ctx.drawImage(img, 0, 0, desiredWidth, desiredHeight);
        };

        img.id = uavName;
        img.style.display = "none";
        body.appendChild(img);

        if(!this.colorSaved) {
            this.backgroundColorWhite = div.style.backgroundColor;
            this.borderColorBlack = canvas.style.borderColor;
            this.colorSaved = true;
        } else {
            for(let i = 0; i < this.removedPics.length; i++) {
                if(this.removedPics.at(i).localeCompare(uavName) == 0) {
                    div.style.backgroundColor = this.backgroundColorGrey;
                    div.style.borderColor = this.borderColorRed;
                    canvas.style.borderColor = this.borderColorRed;
                    return;
                }
                div.style.backgroundColor = this.backgroundColorWhite;
                div.style.borderColor = this.borderColorBlack;
                canvas.style.borderColor = this.borderColorBlack;
            }
        }
    }

    async getPicsUrlsAndNames() {
        this.currendPics = null;
        this.currendPics = new Array();
        for (let i = 1; i <= this.checkCount; i++) {
            this.checkAndSavePicUrl(i);
        }
        while(!this.checkComplet) {
            await new Promise(resolve => setTimeout(resolve, 10));
        }
        this.updateRemovedPics();
    }

    updateRemovedPics() {
        var findMatch;
        var findRemoved;
        for(let i = 0; i < this.uavNames.length; i++) {
            findMatch = false;
            for(let i2 = 0; i2 < this.currendPics.length; i2++) {
                if(this.currendPics.at(i2).localeCompare(this.uavNames.at(i)) == 0) {
                    findMatch = true;
                }
            }
            if(!findMatch) {
                findRemoved = false;
                for(let i2 = 0; i2 < this.removedPics.length; i2++) {
                    if(this.uavNames.at(i).localeCompare(this.removedPics.at(i2)) == 0) {
                        findRemoved = true;
                    }
                }
                if(!findRemoved) {
                    this.removedPics.push(this.uavNames.at(i));
                }
            }
        }
        for(let i = 0; i < this.removedPics.length; i++) {
            for(let i2 = i+1; i2 < this.removedPics.length; i2++) {
                if(this.removedPics.at(i).localeCompare(this.removedPics.at(i2)) == 0) {
                    this.removedPics.splice(i2, 1);
                }
            }
            for(let i2 = 0; i2 < this.currendPics.length; i2++) {
                if(this.currendPics.at(i2).localeCompare(this.removedPics.at(i)) == 0) {
                    this.removedPics.splice(i, 1);
                }
            }
        }
        //console.log(this.removedPics);
    }

    checkAndSavePicUrl(picNameInd) {
        const constThis = this;
        const picId = picNameInd;
        var picUrl = "img/uav" + picNameInd.toString() + ".png";
        this.getPicUrlOrNull(picUrl).then(validUrl => {
            if (picId == constThis.checkCount) {
                constThis.checkComplet = true;
            }
            if (validUrl != null) {
                constThis.currendPics.push("uav" + picId);
                for(let i = 0; i < constThis.picsUrls.length; i++) {
                    if(constThis.picsUrls.at(i).localeCompare(validUrl) == 0) {
                        return;
                    }
                }
                constThis.findNewPicture = true;
                constThis.picsUrls.push(validUrl);
                constThis.uavNames.push("uav" + picId);
            }
        });
    }

    async getPicUrlOrNull(url) {
        return new Promise((resolve, reject) => {
            const img = new Image();
            img.src = url;
            img.onload = () => resolve(url);
            img.onerror = () => {
                reject(`image not found for url ${url}`)
            };
        }).catch(() => {
            return null;
        });
    }

    async outputTextInfo(infoText, clear) {
        var tArea = document.getElementById('text_infos_output');
        if (clear) {
            tArea.value = infoText;
        } else {
            var oldTxt = tArea.value;
            tArea.value = oldTxt + "\r\n" + infoText;
        }
        tArea.scrollTop = tArea.scrollHeight;
    }
}