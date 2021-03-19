// Extraire les données du string
const tryParse = function(str) 
	{
		try 
		{
			JSON.parse(str);
		} 
		catch (e) 
		{
		return false;
		}
		return JSON.parse(str);
	}

module.exports = {
	tryParse,
	}