var Log = {
	debug: true,
	
	// Log.d(logData1, logData2, ...)
	//  --> console.log( getLogHead(), logData1, logData2, ...)
	// 
	// @comment Using bind and property accesser
	// @see http://ejohn.org/blog/javascript-getters-and-setters/
	get d() {
		if ( !this.debug ) return this._emptyFunc;
		return console.log.bind( console, this._getLogHeader() );
	},

	// output error info
	get e() { 
		return console.error.bind( console, this._getLogHeader() );
	},

	// get current time in 01/31 23:59:59.999 format
	_getLogHeader : function () {
		var millisec = Date.now();
		this._dtNow.setTime( millisec );
		return '[' + this._dtNow.toLocaleString().slice( 5 ) + '.' + ('000' + millisec).slice( -3 ) + '] ';
	},
	_dtNow: new Date(),
	_emptyFunc: function() {}
};

function getVal( id ) {
	if ( typeof id !== 'string' || !id instanceof String ) {
		return "";
	}
	return document.getElementById(id).value.trim();
}

function getID( id ) {
	if ( typeof id !== 'string' || !id instanceof String ) {
		//console.log( arguments.callee.name + " : 문자열이 아닙니다." );
		return null;
	}
	return document.getElementById(id);
}

function getCLS( cls ) {
	if ( typeof cls !== 'string' || !cls instanceof String ) {
		//console.log( arguments.callee.name + " : 문자열이 아닙니다." );
		return null;
	}
	return document.getElementsByClassName(cls);
}

function mvNew( strUrl ) {
	window.open(strUrl, '_blank');
}

function mv( url ) {
	window.location.href = url;	
}


function checkEmail( email ) {
	if ( email == "" )
		return false;
	
	var re = /^[a-zA-Z0-9.!#$%&'*+/=?^_`{|}~-]+@[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?(?:\.[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?)*$/;
	return re.test(email);
}

function checkNumber( number ) {
	if ( number == "" )
		return true;
	
	var re = /^\d+$/;
	return re.test(number); 
}

// cookie
function setCookie(cname, cvalue, exdays) {
    var d = new Date();
    d.setTime(d.getTime() + (exdays * 24 * 60 * 60 * 1000));
    var expires = "expires="+d.toUTCString();
    document.cookie = cname + "=" + cvalue + ";" + expires + ";path=/";
}

function getCookie(cname) {
    var name = cname + "=";
    var ca = document.cookie.split(';');
    for(var i = 0; i < ca.length; i++) {
        var c = ca[i];
        while (c.charAt(0) == ' ') {
            c = c.substring(1);
        }
        if (c.indexOf(name) == 0) {
            return c.substring(name.length, c.length);
        }
    }
    return "";
}

function checkCookie() {
    var user = getCookie("username");
    if (user != "") {
        alert("Welcome again " + user);
    } else {
        user = prompt("Please enter your name:", "");
        if (user != "" && user != null) {
            setCookie("username", user, 365);
        }
    }
}

var getTimeStamp = function () {
	var leadingZeros = function(n, digits) {
		var zero = '';
		n = n.toString();

		if ( n.length < digits ) {
			for (i = 0; i < digits - n.length; i++)
				zero += '0';
		}
  		return zero + n;
	}
	
	var d = new Date();

	var s =
	leadingZeros(d.getFullYear(), 4) + '-' +
	leadingZeros(d.getMonth() + 1, 2) + '-' +
	leadingZeros(d.getDate(), 2) + ' ' +
	
	leadingZeros(d.getHours(), 2) + ':' +
	leadingZeros(d.getMinutes(), 2) + ':' +
	leadingZeros(d.getSeconds(), 2);

	return s;
};

var showHide = function(id, bShowHide) {
	var val = '';
	if ( !bShowHide ) val = 'none' 
	document.getElementById(id).style.display = val;
};

var rmOpt = function (id) {
    var select = document.getElementById(id);
    select.options[select.selectedIndex] = null;
};

var setSelectItem = function(id, v) {
    var s = document.getElementById(id);
    s.options[s.selectedIndex].innerText = v;
};

var rmSelectedOpts = function (id) {
    var select = document.getElementById(id);
    var len = select.options.length;
    while ( len-- ) {
    	if ( select.options[len].selected ) {
    		select.options[len] = null;
    	}
    }
};

var selectToArray = function (id) {
	var r = []; // {text: '', value: ''}
    var select = document.getElementById(id);
    
    var len = select.options.length;
    for ( var i = 0 ; i < len ; i++ ) {
    	r.push({value: select.options[i].value, text: select.options[i].innerText});
    }
    return r;
};

var rmAllOpts = function (id) {
    var select = document.getElementById(id);
    select.options.length = 0;
};

var getSelectedText = function(id) {
    var elt = document.getElementById(id);

    if ( elt.selectedIndex == -1 )
        return null;

    return elt.options[elt.selectedIndex].text;
};

var selectCount = function ( id ) {
	return  document.getElementById(id).options.length;
};

var isSelectVal = function ( strSelectID, val ) {
	var IsExists = false;
	var ddloption =  document.getElementById(strSelectID).options;
	var len = ddloption.length;
	for ( var i = 0 ; i < len ; i++ ) {
		if ( ddloption[i].value === val ) {
			IsExists = true;
			break;
		}
	}
	return IsExists;
};

var isSelectTxt = function ( strSelectID, val ) {
	var IsExists = false;
	var ddloption =  document.getElementById(strSelectID).options;
	var len = ddloption.length;
	for ( var i = 0 ; i < len ; i++ ) {
		if ( ddloption[i].innerText === val ) {
			IsExists = true;
			break;
		}
	}
	return IsExists;
};

var queryString = function (name) {
    var match = RegExp('[?&]' + name + '=([^&]*)').exec(window.location.search);
    return match && decodeURIComponent(match[1].replace(/\+/g, ' '));
};

var leadingZeros = function(n, digits) {
	var zero = '';
	n = n.toString();

	if ( n.length < digits ) {
		for (i = 0; i < digits - n.length; i++)
			zero += '0';
	}
	  return zero + n;
};

var utcToLoc = function ( strDateTime ) {
  	var d = new Date(strDateTime); // utc -> local
	// var d = new Date();

	var s =
	leadingZeros(d.getFullYear(), 4) + '-' +
	leadingZeros(d.getMonth() + 1, 2) + '-' +
	leadingZeros(d.getDate(), 2) + ' ' +
	
	leadingZeros(d.getHours(), 2) + ':' +
	leadingZeros(d.getMinutes(), 2) + ':' +
	leadingZeros(d.getSeconds(), 2);

	return s;
};

var utcToLocDate = function ( strDateTime ) {
	var d = new Date(strDateTime); // utc -> local
	var s =	leadingZeros(d.getFullYear(), 4) + '-' +
	leadingZeros(d.getMonth() + 1, 2) + '-' +
	leadingZeros(d.getDate(), 2);
	
	return s;
};

var strToDatetime = function(strInput, format) { 
	var normalized      = strInput.replace(/[^a-zA-Z0-9]/g, '-');
	var normalizedFormat= format.toLowerCase().replace(/[^a-zA-Z0-9]/g, '-');
	var formatItems     = normalizedFormat.split('-');
	var dateItems       = normalized.split('-');
	
	var monthIndex  = formatItems.indexOf("mm");
	var dayIndex    = formatItems.indexOf("dd");
	var yearIndex   = formatItems.indexOf("yyyy");
	var hourIndex     = formatItems.indexOf("hh");
	var minutesIndex  = formatItems.indexOf("ii");
	var secondsIndex  = formatItems.indexOf("ss");
	
	var today = new Date();
	
	var year  = yearIndex>-1  ? dateItems[yearIndex]    : today.getFullYear();
	var month = monthIndex>-1 ? dateItems[monthIndex]-1 : today.getMonth()-1;
	var day   = dayIndex>-1   ? dateItems[dayIndex]     : today.getDate();
	
	var hour    = hourIndex>-1      ? dateItems[hourIndex]    : today.getHours();
	var minute  = minutesIndex>-1   ? dateItems[minutesIndex] : today.getMinutes();
	var second  = secondsIndex>-1   ? dateItems[secondsIndex] : today.getSeconds();
	
	return new Date(year, month, day, hour, minute, second);
};

var timestampToStr = function( ts ){
	var a = new Date(ts);
	var year = a.getFullYear();
	var month = a.getMonth()+1;
	var d = a.getDate();
	var h = a.getHours();
	var m = a.getMinutes();
	var s = a.getSeconds();
	//var time = d + ' ' + month + ' ' + year + ' ' + h + ':' + m + ':' + s;
	var time = year + '-' + month + '-' + d + ' ' + h + ':' + m + ':' + s;
	return time;
}

var locToUtc = function (strLocalDateTime) {
	if ( strLocalDateTime == "" )
		return "";
	return strToDatetime(strLocalDateTime, "yyyy-mm-dd hh:ii:ss").toISOString();
};

var sha256 = function (ascii) {
	if (ascii == '') return '';
    function rightRotate(value, amount) {
        return (value>>>amount) | (value<<(32 - amount));
    };
    var mathPow = Math.pow;
    var maxWord = mathPow(2, 32);
    var lengthProperty = 'length'
    var i, j;
    var result = ''
    var words = [];
    var asciiBitLength = ascii[lengthProperty]*8;
    var hash = sha256.h = sha256.h || [];
    var k = sha256.k = sha256.k || [];
    var primeCounter = k[lengthProperty];
    var isComposite = {};
    for (var candidate = 2; primeCounter < 64; candidate++) {
        if (!isComposite[candidate]) {
            for (i = 0; i < 313; i += candidate) {
                isComposite[i] = candidate;
            }
            hash[primeCounter] = (mathPow(candidate, .5)*maxWord)|0;
            k[primeCounter++] = (mathPow(candidate, 1/3)*maxWord)|0;
        }
    }
    ascii += '\x80'
    while (ascii[lengthProperty]%64 - 56) ascii += '\x00'
    for (i = 0; i < ascii[lengthProperty]; i++) {
        j = ascii.charCodeAt(i);
        if (j>>8) return;
        words[i>>2] |= j << ((3 - i)%4)*8;
    }
    words[words[lengthProperty]] = ((asciiBitLength/maxWord)|0);
    words[words[lengthProperty]] = (asciiBitLength)    
    for (j = 0; j < words[lengthProperty];) {
        var w = words.slice(j, j += 16);
        var oldHash = hash;
        hash = hash.slice(0, 8);
        
        for (i = 0; i < 64; i++) {
            var i2 = i + j;
            var w15 = w[i - 15], w2 = w[i - 2];
            var a = hash[0], e = hash[4];
            var temp1 = hash[7]
                + (rightRotate(e, 6) ^ rightRotate(e, 11) ^ rightRotate(e, 25))
                + ((e&hash[5])^((~e)&hash[6]))
                + k[i]
                + (w[i] = (i < 16) ? w[i] : (
                        w[i - 16]
                        + (rightRotate(w15, 7) ^ rightRotate(w15, 18) ^ (w15>>>3))
                        + w[i - 7]
                        + (rightRotate(w2, 17) ^ rightRotate(w2, 19) ^ (w2>>>10))
                    )|0
                );
            var temp2 = (rightRotate(a, 2) ^ rightRotate(a, 13) ^ rightRotate(a, 22))
                + ((a&hash[1])^(a&hash[2])^(hash[1]&hash[2]));
            
            hash = [(temp1 + temp2)|0].concat(hash);
            hash[4] = (hash[4] + temp1)|0;
        }
        
        for (i = 0; i < 8; i++) {
            hash[i] = (hash[i] + oldHash[i])|0;
        }
    }

    for (i = 0; i < 8; i++) {
        for (j = 3; j + 1; j--) {
            var b = (hash[i]>>(j*8))&255;
            result += ((b < 16) ? 0 : '') + b.toString(16);
        }
    }
    return result;
};

var modalAlert = function(ct) {
	bootbox.alert({
		title: 'Information',
	    message: ct,
	    size: 'small', 
	    buttons: {
	    	ok: {
	    		label: 'OK',
	    		className: 'btn-primary'
	    	}
	    },
	    callback: function(result) {
	    	if (result != undefined) 
	    		console.log('modalAlert() result:', result);
	    }
	});
};

var modalConfirm = function(msg, cb) {
    bootbox.confirm({
        title: 'Question',
        message: msg,
        buttons: {
            confirm: {
                label: 'Yes',
                className: 'btn-primary'
            },
            cancel: {
                label: 'No',
                className: 'btn-primary'
            }
        },
        callback: cb,
    });
};

var validJSON = function(input) {
	try {
		JSON.parse( input );
	} catch (e) {
		modalAlert('JSON is invalid format.');
		return false;
	}	
	return true;
};

var guid=function() {function s4() {return ((1 + Math.random()) * 0x10000 | 0).toString(16).substring(1);}
return s4()+s4()+'-'+s4()+'-'+s4()+'-'+s4()+'-'+s4()+s4()+s4();};

var getTimestampBySec=function(){return Math.floor(+ new Date()/1000);}
var getTimestamp=function(){return Math.floor(+ new Date());}

if ( !window.URLSearchParams ) {
    window.URLSearchParams = function URLSearchParams() {
        this.params = {};
    };
    window.URLSearchParams.prototype = {
        set: function (key, value) {
            this.params[key] = value;
        },
        append: function (key, value) {
            this.params[key] = value;
        },        
        get: function (key) {
            return this.params[key];
        },
        has: function (key) {
            return this.params.hasOwnProperty(key);
        },
        delete: function (key) {
            delete this.params[key];
        },
        toString: function () {
            return Object.keys(this.params).map(function (param) {
                return param + '=' + encodeURIComponent(this.params[param]);
            }, this).join('&');
        },
    };
};

var scrollSave=function(id){localStorage.setItem(id,document.documentElement.scrollTop);};
var scrollRestore=function(id){var p=localStorage.getItem(id,0);if(p)document.documentElement.scrollTop=p;};

var setStorageS = function(k,v) {
	if ( typeof(Storage) === "undefined" ) {
		alert('Storage unsupported browser!!');
		return;
	}
	sessionStorage.setItem(k,v);
};

var getStorageS = function(k) {
	if ( typeof(Storage) === "undefined" ) {
		alert('Storage unsupported browser!!');
		return;
	}
	return sessionStorage.getItem(k)
};

var setStorageL = function(k,v) {
	if ( typeof(Storage) === "undefined" ) {
		alert('Storage unsupported browser!!');
		return;
	}
	localStorage.setItem(k,v);
};

var getStorageL = function(k) {
	if ( typeof(Storage) === "undefined" ) {
		alert('Storage unsupported browser!!');
		return;
	}
	return localStorage.getItem(k)
};

var getSelectRadio = function(n) {
    var result = '';
    var options = document.getElementsByName(n);
    if (options) {
        for (var i = 0; i < options.length; i++) {
            if (options[i].checked){
                result = options[i].value;
                console.log('selected item : ', options[i].value);
                break;
            }
        }
    }
    return result;
};

var setSelectRadio = function(n, v) {
	var options = document.getElementsByName(n);
    if (options) {
        for (var i = 0; i < options.length; i++) {
            if (options[i].value == v){
                options[i].checked = true;
                break;
            }
        }
    }
};

var hFS = function ( bytes ) {
	var si = true;
    var thresh = si ? 1000 : 1024;
    if ( Math.abs(bytes) < thresh ) {
        return bytes + ' B';
    }
    var units = si
    ? ['KB','MB','GB','TB','PB','EB','ZB','YB']
    : ['KiB','MiB','GiB','TiB','PiB','EiB','ZiB','YiB'];
    var u = -1;
    do {
        bytes /= thresh;
        ++u;
    } while(Math.abs(bytes) >= thresh && u < units.length - 1);
    
    return bytes.toFixed(1)+' '+units[u];
};

var fmtPhone=function(num){return num.replace(/(^02.{0}|^01.{1}|[0-9]{3})([0-9]+)([0-9]{4})/,"$1-$2-$3");};
//
// axios.interceptors.request.use(function (config) {
// 	  if(getID('_csrf') != null) {
// 		  var _csrf = getID('_csrf').value;
// 		  config.headers['x-csrf-token'] = _csrf;
// 	  }
// 	  return config;
// 	}
// );

var isEMPTY=function(i){
	return (i==null||typeof i=='undefined')?true:false;
};

var isEMPSTR=function(i){
	return (i==null||typeof i=='undefined'||i=='')?true:false;
};