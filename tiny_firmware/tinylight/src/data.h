/*
 * data.h
 *
 * Created: 22.03.2014 10:48:04
 *  Author: Folfy
 */ 


#ifndef DATA_H_
#define DATA_H_

/*	Gamma correction table for gamma=2.2	*/
/*	y=1020*(x/255)^gamma					*/

// const uint16_t gamma_10b[256] =
// {
// /*	0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F		*/
// 	0		,0		,0		,0		,0		,0		,0		,0		,1		,1		,1		,1		,1		,1		,2		,2		,	//	0
// 	2		,3		,3		,3		,4		,4		,5		,5		,6		,6		,7		,7		,8		,9		,9		,10		,	//	1
// 	11		,11		,12		,13		,14		,15		,16		,16		,17		,18		,19		,20		,21		,23		,24		,25		,	//	2
// 	26		,27		,28		,30		,31		,32		,34		,35		,36		,38		,39		,41		,42		,44		,46		,47		,	//	3
// 	49		,51		,52		,54		,56		,58		,60		,61		,63		,65		,67		,69		,71		,73		,76		,78		,	//	4
// 	80		,82		,84		,87		,89		,91		,94		,96		,98		,101	,103	,106	,109	,111	,114	,117	,	//	5
// 	119		,122	,125	,128	,130	,133	,136	,139	,142	,145	,148	,151	,155	,158	,161	,164	,	//	6
// 	167		,171	,174	,177	,181	,184	,188	,191	,195	,198	,202	,206	,209	,213	,217	,221	,	//	7
// 	225		,228	,232	,236	,240	,244	,248	,252	,257	,261	,265	,269	,274	,278	,282	,287	,	//	8
// 	291		,295	,300	,304	,309	,314	,318	,323	,328	,333	,337	,342	,347	,352	,357	,362	,	//	9
// 	367		,372	,377	,382	,387	,393	,398	,403	,408	,414	,419	,425	,430	,436	,441	,447	,	//	A
// 	452		,458	,464	,470	,475	,481	,487	,493	,499	,505	,511	,517	,523	,529	,535	,542	,	//	B
// 	548		,554	,561	,567	,573	,580	,586	,593	,599	,606	,613	,619	,626	,633	,640	,647	,	//	C
// 	653		,660	,667	,674	,681	,689	,696	,703	,710	,717	,725	,732	,739	,747	,754	,762	,	//	D
// 	769		,777	,784	,792	,800	,807	,815	,823	,831	,839	,847	,855	,863	,871	,879	,887	,	//	E
// 	895		,903	,912	,920	,928	,937	,945	,954	,962	,971	,979	,988	,997	,1003	,1012	,1020		//	F
// };

const uint16_t root_10[256] PROGMEM =
{
/*	0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F		*/
	0		,37654	,40357	,42027	,43253	,44229	,45043	,45743	,46358	,46907	,47404	,47858	,48276	,48664	,49026	,49366	,	//	0
	49685	,49987	,50274	,50546	,50806	,51055	,51293	,51521	,51741	,51953	,52157	,52354	,52545	,52730	,52909	,53082	,	//	1
	53251	,53415	,53575	,53731	,53882	,54030	,54174	,54315	,54453	,54588	,54719	,54848	,54974	,55098	,55219	,55338	,	//	2
	55455	,55569	,55682	,55792	,55901	,56007	,56112	,56215	,56316	,56416	,56514	,56611	,56706	,56800	,56892	,56984	,	//	3
	57073	,57162	,57249	,57335	,57420	,57504	,57587	,57669	,57750	,57829	,57908	,57986	,58063	,58139	,58214	,58288	,	//	4
	58361	,58434	,58506	,58576	,58647	,58716	,58785	,58853	,58920	,58987	,59053	,59118	,59183	,59247	,59310	,59373	,	//	5
	59435	,59497	,59558	,59618	,59678	,59738	,59796	,59855	,59913	,59970	,60027	,60083	,60139	,60195	,60250	,60304	,	//	6
	60358	,60412	,60465	,60518	,60571	,60623	,60674	,60725	,60776	,60827	,60877	,60927	,60976	,61025	,61073	,61122	,	//	7
	61170	,61217	,61265	,61312	,61358	,61405	,61451	,61496	,61542	,61587	,61632	,61676	,61720	,61764	,61808	,61851	,	//	8
	61894	,61937	,61980	,62022	,62064	,62106	,62148	,62189	,62230	,62271	,62311	,62352	,62392	,62432	,62471	,62511	,	//	9
	62550	,62589	,62628	,62666	,62705	,62743	,62781	,62818	,62856	,62893	,62930	,62967	,63004	,63041	,63077	,63113	,	//	A
	63149	,63185	,63220	,63256	,63291	,63326	,63361	,63396	,63430	,63465	,63499	,63533	,63567	,63601	,63634	,63668	,	//	B
	63701	,63734	,63767	,63800	,63832	,63865	,63897	,63929	,63962	,63993	,64025	,64057	,64088	,64120	,64151	,64182	,	//	C
	64213	,64244	,64274	,64305	,64335	,64366	,64396	,64426	,64456	,64485	,64515	,64545	,64574	,64603	,64633	,64662	,	//	D
	64691	,64719	,64748	,64777	,64805	,64834	,64862	,64890	,64918	,64946	,64974	,65001	,65029	,65057	,65084	,65111	,	//	E
	65138	,65165	,65192	,65219	,65246	,65273	,65299	,65326	,65352	,65379	,65405	,65431	,65457	,65483	,65509	,65535		//	F
};

#endif /* DATA_H_ */