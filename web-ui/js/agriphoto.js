$(document).ready(function(){
	setInterval(function(){
		$.getJSON("http://127.0.0.1:9000/status", function(result){
		    $("#statusmsg").html(result["status"]);
			$("#aclat").html(result["ac"]["lat"]);
			$("#aclon").html(result["ac"]["lon"]);
			$("#acheading").html(result["ac"]["heading"]);
			$("#airspeed").html(result["ac"]["airspeed"]);
			$("#groundspeed").html(result["ac"]["groundspeed"]);
			$("#setspeed").html(result["ac"]["setspeed"]);
			$("#nowinspeed").html(result["ac"]["nowinspeed"]);
			$("#wind").html(result["wind"]);
			$("#winddir").html(result["winddir"]);
			$("#setwind").html(result["setwind"]);
			$("#setwinddir").html(result["setwinddir"]);
			$("#alt").html(result["ac"]["alt"]);
			$("#setalt").html(result["ac"]["setalt"]);
			$("#setx").html(result["ac"]["setx"]);
			$("#sety").html(result["ac"]["sety"]);
			$("#gcslat").html(result["gcs"]["lat"]);
			$("#gcslon").html(result["gcs"]["lon"]);
			$("#gcsheading").html(result["gcs"]["heading"]);
			$("#gcsspeed").html(result["gcs"]["speed"]);
			$("#rate").html(result["rate"]);
			$("#gain_f").html(result["gain_f"]);
			$("#gain_b").html(result["gain_b"]);
			$("#altbase").html(result["altbase"]);
			$("#altamp").html(result["altamp"]);
			$("#altper").html(result["altper"]);
			$("#wp_dist").html(result["wp_dist"]);
		})
			.fail( function() {
				$("#statusmsg").html("Not running");
				$("#statusmsg,#aclat,#aclon,#acheading,\
					#airspeed,#groundspeed,#setspeed,\
					#nowinspeed,#wind,#winddir,#setwind,\
					#setwinddir,#alt,#setalt,#setx,#sety\
					#gcslat,#gcslon,#gcsheading,#gcsspeed\
					#rate,#gain_f,#gain_b,#altbase,#altamp\
					#altper,#wp_dist").html("");

			});
	}, 1000);
});
