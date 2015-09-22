$(document).ready(function(){
	setInterval(function(){
		$.getJSON("http://127.0.0.1:9000/status", function(result){
		    $("#statusmsg").html(result["status"]);
			$("#aclatlon").html(result["aclatlon"]);
			$("#airspeed").html(result["airspeed"]);
			$("#groundspeed").html(result["groundspeed"]);
			$("#setspeed").html(result["setspeed"]);
			$("#nowinspeed").html(result["nowinspeed"]);
			$("#wind").html(result["wind"]);
			$("#setwind").html(result["setwind"]);
			$("#alt").html(result["alt"]);
			$("#setalt").html(result["setalt"]);
			$("#setxy").html(result["setxy"]);
			$("#gcslatlon").html(result["gcslatlon"]);
			$("#gcsspeed").html(result["gcsspeed"]);
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
				$("#aclatlon,#airspeed,#groundspeed,#setspeed,#nowinspeed,#wind,#setwind,#alt,#setalt,#setxy,#gcslatlon,#gcsspeed").html("");

			});
	}, 1000);
});
