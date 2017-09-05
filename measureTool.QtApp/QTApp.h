#pragma once

#include <memory>

class	QTApp
{	
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

private:

	QTApp();

public:

	~QTApp();

public:

	static	QTApp&	GetInstance();

public:

	void	Init(int argc, char** argv);

	int		Run();
};